#!/usr/bin/env python3
"""Merge two monocular ArUco views and report marker 0 in marker 4 coordinates.

OpenCV returns each marker pose as ``T_camera_marker``: points expressed in the
marker frame are transformed into the camera frame. This module uses shared
anchor markers to estimate ``T_camera2_camera1``, then expresses every detected
marker in the marker-4 frame.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
import time
from typing import Any, Iterable, Sequence

import cv2
import numpy as np
from scipy.signal import butter, sosfilt, sosfilt_zi


WORKSPACE_DIR = Path(__file__).resolve().parent.parent
DEFAULT_INTRINSICS_CAM1 = WORKSPACE_DIR / "camera_calibration" / "cam1.yaml"
DEFAULT_INTRINSICS_CAM2 = WORKSPACE_DIR / "camera_calibration" / "cam2.yaml"
DEFAULT_SETTINGS_CAM1 = WORKSPACE_DIR / "camera_calibration" / "cam1_settings.yaml"
DEFAULT_SETTINGS_CAM2 = WORKSPACE_DIR / "camera_calibration" / "cam2_settings.yaml"
DEFAULT_MARKER_SIZE_M = 0.20
DEFAULT_DICTIONARY = "DICT_4X4_50"
DEFAULT_ANCHOR_IDS = (1, 2, 3, 4, 5, 6)
DEFAULT_SHARED_IDS = (3, 4)
EXPECTED_IDS_CAM1 = (1, 2, 3, 4)
EXPECTED_IDS_CAM2 = (3, 4, 5, 6)
LAYOUT_COORDS: dict[int, tuple[float, float, float]] = {
    1: (60.0, -50.0, 0.0),
    2: (0.0, -50.0, 0.0),
    3: (60.0, 0.0, 0.0),
    4: (0.0, 0.0, 0.0),
    5: (60.0, 50.0, 0.0),
    6: (0.0, 50.0, 0.0),
}

# _layout_from_origin compares this against poses_in_origin[...][:3, 3], which is a
# translation in metres (everything in this file - marker_size_m, solvePnP tvecs - is in
# metres). LAYOUT_COORDS above is specified in centimetres, so convert once here.
LAYOUT_COORDS_M: dict[int, tuple[float, float, float]] = {
    marker_id: (x / 100.0, y / 100.0, z / 100.0)
    for marker_id, (x, y, z) in LAYOUT_COORDS.items()
}

CAMERA_SETTING_KEYS: dict[str, int] = {
    "frame width": cv2.CAP_PROP_FRAME_WIDTH,
    "frame height": cv2.CAP_PROP_FRAME_HEIGHT,
    "fps": cv2.CAP_PROP_FPS,
    "brightness": cv2.CAP_PROP_BRIGHTNESS,
    "contrast": cv2.CAP_PROP_CONTRAST,
    "saturation": cv2.CAP_PROP_SATURATION,
    "hue": cv2.CAP_PROP_HUE,
    "gain": cv2.CAP_PROP_GAIN,
    "exposure": cv2.CAP_PROP_EXPOSURE,
    "auto exposure": cv2.CAP_PROP_AUTO_EXPOSURE,
    "buffer size": cv2.CAP_PROP_BUFFERSIZE,
}


@dataclass(frozen=True)
class Intrinsics:
    """Pinhole camera intrinsics and distortion coefficients."""

    camera_matrix: np.ndarray
    distortion: np.ndarray
    image_size: tuple[int, int] | None = None

    def scaled_to_frame(self, frame: np.ndarray) -> "Intrinsics":
        """Return intrinsics scaled to a captured frame's width and height."""

        if self.image_size is None:
            return self

        source_width, source_height = self.image_size
        frame_height, frame_width = frame.shape[:2]
        if source_width <= 0 or source_height <= 0:
            return self
        if source_width == frame_width and source_height == frame_height:
            return self

        scaled = self.camera_matrix.copy()
        x_scale = frame_width / float(source_width)
        y_scale = frame_height / float(source_height)
        scaled[0, 0] *= x_scale
        scaled[0, 2] *= x_scale
        scaled[1, 1] *= y_scale
        scaled[1, 2] *= y_scale
        return Intrinsics(scaled, self.distortion, (frame_width, frame_height))


@dataclass(frozen=True)
class MarkerPose:
    """Pose and corners for one detected marker in one camera."""

    marker_id: int
    camera_index: int
    corners: np.ndarray
    transform_camera_marker: np.ndarray
    rvec: np.ndarray
    tvec: np.ndarray


@dataclass(frozen=True)
class MergeResult:
    """Merged pose result for one synchronized frame pair."""

    timestamp_s: float
    used_shared_ids: tuple[int, ...]
    camera2_to_camera1: np.ndarray | None
    poses_in_camera1: dict[int, np.ndarray]
    poses_in_origin: dict[int, np.ndarray]
    target_in_origin: np.ndarray | None
    layout_from_origin: np.ndarray | None
    poses_in_layout: dict[int, np.ndarray]
    target_in_layout: np.ndarray | None
    target_relative_to_anchors: dict[int, np.ndarray]
    observed_by: dict[int, tuple[int, ...]]
    merge_residuals: dict[int, dict[str, float]]

    @property
    def has_target(self) -> bool:
        """Return whether the target marker pose was estimated."""

        return self.target_in_layout is not None or self.target_in_origin is not None


def _topic_from_settings(
    settings: dict[str, Any],
    *,
    fallback_topic: str,
) -> str:
    """Return the ROS image topic configured for one camera."""

    raw_topic = settings.get("ros_topic", fallback_topic)
    topic = str(raw_topic).strip() if raw_topic is not None else ""
    return topic or fallback_topic


def validate_camera_settings_pair(
    settings1: dict[str, Any],
    settings2: dict[str, Any],
) -> tuple[str, str]:
    """Ensure both cameras are configured with distinct ROS topics."""

    topic1 = _topic_from_settings(settings1, fallback_topic="/cam1/image_raw")
    topic2 = _topic_from_settings(settings2, fallback_topic="/cam2/image_raw")
    if topic1 == topic2:
        raise ValueError(
            "Camera topic collision: both cameras are configured to publish on "
            f"{topic1!r}. Use different ros_topic values in cam1_settings.yaml "
            "and cam2_settings.yaml."
        )
    return topic1, topic2


def parse_id_list(raw: str | Iterable[int]) -> tuple[int, ...]:
    """Parse a comma-separated marker-id list."""

    if isinstance(raw, str):
        if not raw.strip():
            return ()
        return tuple(int(piece.strip()) for piece in raw.split(",") if piece.strip())
    return tuple(int(value) for value in raw)


def normalize_distortion(distortion: np.ndarray) -> np.ndarray:
    """Return distortion coefficients as a float64 column vector."""

    distortion = np.asarray(distortion, dtype=np.float64)
    if distortion.size == 0:
        return np.zeros((5, 1), dtype=np.float64)
    return distortion.reshape(-1, 1)


def load_intrinsics(path: str | Path) -> Intrinsics:
    """Load camera intrinsics from OpenCV YAML, ROS camera-info YAML, or JSON."""

    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Intrinsics file not found: {path}")

    if path.suffix.lower() == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        camera_matrix = _matrix_from_payload(payload["camera_matrix"])
        distortion_payload = payload.get(
            "distortion_coefficients",
            payload.get("dist_coeffs", payload.get("distortion")),
        )
        distortion = _matrix_from_payload(distortion_payload)
        return Intrinsics(
            camera_matrix,
            normalize_distortion(distortion),
            _image_size_from_payload(payload),
        )

    camera_matrix, distortion, image_size = _load_opencv_yaml(path)
    if camera_matrix is not None and distortion is not None:
        return Intrinsics(camera_matrix, normalize_distortion(distortion), image_size)

    camera_matrix, distortion, image_size = _load_ros_yaml(path)
    if camera_matrix is not None and distortion is not None:
        return Intrinsics(camera_matrix, normalize_distortion(distortion), image_size)

    raise ValueError(
        f"{path} must contain camera_matrix and distortion_coefficients."
    )


def load_camera_settings(path: str | Path | None) -> dict[str, Any]:
    """Load simple camera settings from YAML/JSON-like files."""

    if path is None:
        return {}

    path = Path(path)
    if not path.exists():
        return {}

    if path.suffix.lower() == ".json":
        payload = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(payload, dict):
            raise ValueError(f"{path} must contain a JSON object.")
        return payload

    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError:
        yaml = None  # type: ignore[assignment]

    if yaml is not None:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
        if payload is None:
            return {}
        if not isinstance(payload, dict):
            raise ValueError(f"{path} must contain key/value pairs.")
        return payload

    settings: dict[str, Any] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or ":" not in line:
            continue
        key, value = line.split(":", 1)
        settings[key.strip()] = _parse_scalar_value(value.strip())
    return settings


def _parse_scalar_value(value: str) -> Any:
    """Parse a scalar string from a simple YAML-like settings file."""

    if not value:
        return ""
    lowered = value.lower()
    if lowered in {"true", "false"}:
        return lowered == "true"
    try:
        if any(marker in value for marker in (".", "e", "E")):
            return float(value)
        return int(value)
    except ValueError:
        return value


def _matrix_from_payload(payload: Any) -> np.ndarray:
    """Read a matrix from either a nested camera-info mapping or a raw list."""

    if isinstance(payload, dict) and "data" in payload:
        data = np.asarray(payload["data"], dtype=np.float64)
        rows = int(payload.get("rows", 1))
        cols = int(payload.get("cols", data.size))
        return data.reshape(rows, cols)
    return np.asarray(payload, dtype=np.float64)


def _image_size_from_payload(payload: dict[str, Any]) -> tuple[int, int] | None:
    """Read image size from JSON/ROS-style camera payloads."""

    width = payload.get("image_width", payload.get("width"))
    height = payload.get("image_height", payload.get("height"))
    if width is None or height is None:
        return None
    return int(width), int(height)


def _load_opencv_yaml(
    path: Path,
) -> tuple[np.ndarray | None, np.ndarray | None, tuple[int, int] | None]:
    """Try OpenCV FileStorage YAML parsing."""

    try:
        storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    except (cv2.error, SystemError):
        return None, None, None
    if not storage.isOpened():
        return None, None, None

    try:
        camera_matrix = storage.getNode("camera_matrix").mat()
        distortion = storage.getNode("distortion_coefficients").mat()
        if distortion is None:
            distortion = storage.getNode("dist_coeffs").mat()
        width_node = storage.getNode("image_width")
        height_node = storage.getNode("image_height")
        image_size = None
        if not width_node.empty() and not height_node.empty():
            image_size = (int(width_node.real()), int(height_node.real()))
    except cv2.error:
        return None, None, None
    finally:
        storage.release()

    if camera_matrix is None or distortion is None:
        return None, None, None
    return camera_matrix.astype(np.float64), distortion.astype(np.float64), image_size


def _load_ros_yaml(
    path: Path,
) -> tuple[np.ndarray | None, np.ndarray | None, tuple[int, int] | None]:
    """Try parsing ROS camera-info YAML with PyYAML when it is available."""

    try:
        import yaml  # type: ignore[import-untyped]
    except ImportError:
        return None, None, None

    try:
        payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    except Exception:
        return None, None, None

    if not isinstance(payload, dict):
        return None, None, None

    camera_payload = payload.get("camera_matrix")
    distortion_payload = payload.get("distortion_coefficients")
    if camera_payload is None or distortion_payload is None:
        return None, None, None

    return (
        _matrix_from_payload(camera_payload),
        _matrix_from_payload(distortion_payload),
        _image_size_from_payload(payload),
    )


def load_aruco_dictionary(dictionary_name: str) -> cv2.aruco.Dictionary:
    """Load an OpenCV ArUco dictionary by constant name."""

    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is unavailable; install opencv-contrib-python."
        )
    if not hasattr(cv2.aruco, dictionary_name):
        raise ValueError(f"Unknown OpenCV ArUco dictionary: {dictionary_name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))


def make_detector_parameters() -> cv2.aruco.DetectorParameters:
    """Create detector parameters across OpenCV 4.x API variants."""

    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return params


def marker_object_points(marker_size_m: float) -> np.ndarray:
    """Return square marker object points for solvePnP IPPE_SQUARE."""

    half = marker_size_m * 0.5
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def transform_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Build a 4x4 transform from OpenCV rotation and translation vectors."""

    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return transform


def invert_transform(transform: np.ndarray) -> np.ndarray:
    """Invert a rigid 4x4 transform."""

    inverse = np.eye(4, dtype=np.float64)
    rotation = transform[:3, :3]
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ transform[:3, 3]
    return inverse


def rotation_matrix_to_quaternion_xyzw(rotation: np.ndarray) -> np.ndarray:
    """Convert a rotation matrix to a normalized quaternion in x, y, z, w order."""

    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (rotation[2, 1] - rotation[1, 2]) / scale
        qy = (rotation[0, 2] - rotation[2, 0]) / scale
        qz = (rotation[1, 0] - rotation[0, 1]) / scale
    else:
        diag_index = int(np.argmax(np.diag(rotation)))
        if diag_index == 0:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / scale
            qx = 0.25 * scale
            qy = (rotation[0, 1] + rotation[1, 0]) / scale
            qz = (rotation[0, 2] + rotation[2, 0]) / scale
        elif diag_index == 1:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / scale
            qx = (rotation[0, 1] + rotation[1, 0]) / scale
            qy = 0.25 * scale
            qz = (rotation[1, 2] + rotation[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / scale
            qx = (rotation[0, 2] + rotation[2, 0]) / scale
            qy = (rotation[1, 2] + rotation[2, 1]) / scale
            qz = 0.25 * scale

    quaternion = np.array([qx, qy, qz, qw], dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
    return quaternion / norm


def quaternion_xyzw_to_rotation_matrix(quaternion: np.ndarray) -> np.ndarray:
    """Convert a normalized x, y, z, w quaternion to a rotation matrix."""

    qx, qy, qz, qw = np.asarray(quaternion, dtype=np.float64)
    return np.array(
        [
            [
                1.0 - 2.0 * (qy * qy + qz * qz),
                2.0 * (qx * qy - qz * qw),
                2.0 * (qx * qz + qy * qw),
            ],
            [
                2.0 * (qx * qy + qz * qw),
                1.0 - 2.0 * (qx * qx + qz * qz),
                2.0 * (qy * qz - qx * qw),
            ],
            [
                2.0 * (qx * qz - qy * qw),
                2.0 * (qy * qz + qx * qw),
                1.0 - 2.0 * (qx * qx + qy * qy),
            ],
        ],
        dtype=np.float64,
    )


def average_transforms(transforms: Sequence[np.ndarray]) -> np.ndarray:
    """Average translations and quaternions from several rigid transforms."""

    if not transforms:
        raise ValueError("Cannot average an empty transform list.")
    if len(transforms) == 1:
        return transforms[0].copy()

    translations = np.stack([transform[:3, 3] for transform in transforms])
    quaternions = [
        rotation_matrix_to_quaternion_xyzw(transform[:3, :3])
        for transform in transforms
    ]

    reference = quaternions[0]
    aligned = []
    for quaternion in quaternions:
        if float(np.dot(reference, quaternion)) < 0.0:
            quaternion = -quaternion
        aligned.append(quaternion)

    mean_quaternion = np.mean(np.stack(aligned), axis=0)
    norm = float(np.linalg.norm(mean_quaternion))
    if norm == 0.0:
        mean_quaternion = reference
    else:
        mean_quaternion /= norm

    averaged = np.eye(4, dtype=np.float64)
    averaged[:3, :3] = quaternion_xyzw_to_rotation_matrix(mean_quaternion)
    averaged[:3, 3] = np.mean(translations, axis=0)
    return averaged


def planar_similarity_transform_from_points(
    source_points: Sequence[np.ndarray],
    target_points: Sequence[np.ndarray],
) -> np.ndarray:
    """Estimate a planar similarity transform from observed floor points to layout points."""

    if len(source_points) != len(target_points):
        raise ValueError("source_points and target_points must have the same length.")
    if len(source_points) < 2:
        raise ValueError("Need at least two point correspondences.")

    source = np.stack([np.asarray(point, dtype=np.float64).reshape(3) for point in source_points])
    target = np.stack([np.asarray(point, dtype=np.float64).reshape(3) for point in target_points])

    source_xy = source[:, :2]
    target_xy = target[:, :2]
    source_centroid_xy = np.mean(source_xy, axis=0)
    target_centroid_xy = np.mean(target_xy, axis=0)
    source_centered_xy = source_xy - source_centroid_xy
    target_centered_xy = target_xy - target_centroid_xy

    covariance = source_centered_xy.T @ target_centered_xy
    u_matrix, _singular_values, vt_matrix = np.linalg.svd(covariance)
    rotation_xy = vt_matrix.T @ u_matrix.T
    if np.linalg.det(rotation_xy) < 0.0:
        vt_matrix[-1, :] *= -1.0
        rotation_xy = vt_matrix.T @ u_matrix.T

    source_variance = float(np.sum(source_centered_xy * source_centered_xy))
    if source_variance <= 1e-12:
        raise ValueError("Observed anchor points are degenerate.")
    scale = float(np.sum(_singular_values)) / source_variance
    translation_xy = target_centroid_xy - scale * (rotation_xy @ source_centroid_xy)
    translation_z = float(np.mean(target[:, 2] - source[:, 2]))

    transform = np.eye(4, dtype=np.float64)
    transform[:2, :2] = scale * rotation_xy
    transform[2, 2] = 1.0
    transform[0, 3] = translation_xy[0]
    transform[1, 3] = translation_xy[1]
    transform[2, 3] = translation_z
    return transform


def transform_error(reference: np.ndarray, candidate: np.ndarray) -> dict[str, float]:
    """Return translation and rotation disagreement between two transforms."""

    delta = invert_transform(reference) @ candidate
    translation_m = float(np.linalg.norm(delta[:3, 3]))
    trace = float(np.trace(delta[:3, :3]))
    cosine = max(-1.0, min(1.0, (trace - 1.0) * 0.5))
    rotation_deg = math.degrees(math.acos(cosine))
    return {
        "translation_m": translation_m,
        "rotation_deg": rotation_deg,
    }


def rpy_degrees_from_rotation(rotation: np.ndarray) -> tuple[float, float, float]:
    """Return roll, pitch, yaw in degrees using the XYZ convention."""

    sy = math.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = math.atan2(-rotation[1, 2], rotation[1, 1])
        pitch = math.atan2(-rotation[2, 0], sy)
        yaw = 0.0
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def pose_payload(transform: np.ndarray) -> dict[str, Any]:
    """Convert a transform into JSON-friendly pose fields."""

    rotation = transform[:3, :3]
    rvec, _ = cv2.Rodrigues(rotation)
    roll_deg, pitch_deg, yaw_deg = rpy_degrees_from_rotation(rotation)
    return {
        "translation_m": [float(value) for value in transform[:3, 3]],
        "rotation_vector": [float(value) for value in rvec.reshape(3)],
        "quaternion_xyzw": [
            float(value)
            for value in rotation_matrix_to_quaternion_xyzw(rotation)
        ],
        "rpy_deg": [roll_deg, pitch_deg, yaw_deg],
        "matrix": [[float(value) for value in row] for row in transform],
    }


def rotation_distance(rvec1: np.ndarray, rvec2: np.ndarray) -> float:
    """Return the angle difference in radians between two rotation vectors."""
    R1, _ = cv2.Rodrigues(rvec1)
    R2, _ = cv2.Rodrigues(rvec2)
    trace = np.trace(R1.T @ R2)
    return math.acos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))


class ArucoPoseDetector:
    """Detect square ArUco marker SE(3) poses from one camera frame."""

    def __init__(self, marker_size_m: float, dictionary_name: str) -> None:
        if marker_size_m <= 0.0:
            raise ValueError("marker_size_m must be positive.")
        self.marker_size_m = float(marker_size_m)
        self.dictionary = load_aruco_dictionary(dictionary_name)
        self.parameters = make_detector_parameters()
        self.object_points = marker_object_points(self.marker_size_m)
        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary,
                self.parameters,
            )
        else:
            self.detector = None
        self._last_rvecs: dict[int, np.ndarray] = {}

    def detect(
        self,
        frame: np.ndarray,
        intrinsics: Intrinsics,
        camera_index: int,
        allowed_ids: Iterable[int] | None = None,
    ) -> dict[int, MarkerPose]:
        """Return marker poses keyed by marker id."""

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.detector is not None:
            corners, ids, _rejected = self.detector.detectMarkers(gray)
        else:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters,
            )

        if ids is None:
            return {}

        allowed = None if allowed_ids is None else set(int(value) for value in allowed_ids)
        detections: dict[int, MarkerPose] = {}
        for index, raw_marker_id in enumerate(ids.reshape(-1)):
            marker_id = int(raw_marker_id)
            if allowed is not None and marker_id not in allowed:
                continue

            image_points = corners[index].reshape(4, 2).astype(np.float64)
            ok, rvec, tvec = self._solve_marker_pose(image_points, intrinsics, marker_id=marker_id)
            if not ok:
                continue

            transform = transform_from_rvec_tvec(rvec, tvec)
            detections[marker_id] = MarkerPose(
                marker_id=marker_id,
                camera_index=camera_index,
                corners=image_points.astype(np.float32),
                transform_camera_marker=transform,
                rvec=np.asarray(rvec, dtype=np.float64).reshape(3, 1),
                tvec=np.asarray(tvec, dtype=np.float64).reshape(3, 1),
            )

        return detections

    def _solve_marker_pose(
        self,
        image_points: np.ndarray,
        intrinsics: Intrinsics,
        marker_id: int | None = None,
    ) -> tuple[bool, np.ndarray, np.ndarray]:
        """Solve one square marker pose, preferring IPPE_SQUARE when available."""

        flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)

        # Use solvePnPGeneric to get both candidates for IPPE_SQUARE to resolve ambiguity
        if flags == cv2.SOLVEPNP_IPPE_SQUARE:
            ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(
                self.object_points,
                image_points,
                intrinsics.camera_matrix,
                intrinsics.distortion,
                flags=flags,
            )
            if ok and len(rvecs) > 0:
                # If we have a cached previous rotation for this marker, select the closest candidate
                if marker_id is not None and marker_id in self._last_rvecs:
                    last_rvec = self._last_rvecs[marker_id]
                    best_idx = 0
                    min_dist = float("inf")
                    for idx, rvec in enumerate(rvecs):
                        dist = rotation_distance(rvec, last_rvec)
                        if dist < min_dist:
                            min_dist = dist
                            best_idx = idx
                    rvec = rvecs[best_idx]
                    tvec = tvecs[best_idx]
                else:
                    # Otherwise default to the first candidate (usually lowest error)
                    rvec = rvecs[0]
                    tvec = tvecs[0]

                # Update cache
                if marker_id is not None:
                    self._last_rvecs[marker_id] = rvec.copy()
                return True, rvec, tvec

        # Fallback to standard iterative solvePnP
        ok, rvec, tvec = cv2.solvePnP(
            self.object_points,
            image_points,
            intrinsics.camera_matrix,
            intrinsics.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if ok and marker_id is not None:
            self._last_rvecs[marker_id] = rvec.copy()
        return ok, rvec, tvec


class ButterworthAxisFilter:
    """Stateful low-pass Butterworth filter for one streaming scalar signal.

    Designed for real-time use: each call to update() advances the filter by exactly
    one sample and returns the filtered value immediately, using second-order-section
    (sos) form for numerical stability over long-running streams.
    """

    def __init__(self, sample_rate_hz: float, cutoff_hz: float, order: int = 2) -> None:
        if sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be positive.")
        nyquist_hz = sample_rate_hz / 2.0
        if not (0.0 < cutoff_hz < nyquist_hz):
            raise ValueError(
                f"cutoff_hz ({cutoff_hz}) must be between 0 and the Nyquist frequency "
                f"({nyquist_hz}) for sample_rate_hz={sample_rate_hz}."
            )
        self.sos = butter(order, cutoff_hz / nyquist_hz, btype="low", output="sos")
        self._zi_template = sosfilt_zi(self.sos)
        self._zi: np.ndarray | None = None

    def reset(self, initial_value: float = 0.0) -> None:
        """Reset internal filter state, seeded so the first output equals initial_value."""

        self._zi = self._zi_template * float(initial_value)

    def update(self, value: float) -> float:
        """Filter one new sample and return the filtered output."""

        if self._zi is None:
            self.reset(value)
        filtered, self._zi = sosfilt(self.sos, [value], zi=self._zi)
        return float(filtered[0])


class PoseButterworthFilter:
    """Filters a planar (x, y, yaw) pose stream, one Butterworth filter per axis.

    Yaw is unwrapped before filtering (and re-wrapped to (-pi, pi] after) so the filter
    never sees a spurious +-2*pi jump when the marker's heading crosses the wraparound
    boundary - a raw Butterworth filter has no notion of angles being circular.
    """

    def __init__(self, sample_rate_hz: float, cutoff_hz: float, order: int = 2) -> None:
        self._filter_x = ButterworthAxisFilter(sample_rate_hz, cutoff_hz, order)
        self._filter_y = ButterworthAxisFilter(sample_rate_hz, cutoff_hz, order)
        self._filter_yaw = ButterworthAxisFilter(sample_rate_hz, cutoff_hz, order)
        self._unwrapped_yaw_prev: float | None = None

    def update(self, x: float, y: float, yaw_rad: float) -> tuple[float, float, float]:
        """Filter one new (x, y, yaw) sample, returning the filtered (x, y, yaw_rad)."""

        if self._unwrapped_yaw_prev is None:
            unwrapped_yaw = yaw_rad
        else:
            delta = yaw_rad - self._unwrapped_yaw_prev
            delta = math.atan2(math.sin(delta), math.cos(delta))
            unwrapped_yaw = self._unwrapped_yaw_prev + delta
        self._unwrapped_yaw_prev = unwrapped_yaw

        filtered_x = self._filter_x.update(x)
        filtered_y = self._filter_y.update(y)
        filtered_yaw = self._filter_yaw.update(unwrapped_yaw)
        filtered_yaw = math.atan2(math.sin(filtered_yaw), math.cos(filtered_yaw))
        return filtered_x, filtered_y, filtered_yaw


def transform_with_planar_pose(
    reference_transform: np.ndarray,
    x: float,
    y: float,
    yaw_rad: float,
) -> np.ndarray:
    """Return a copy of reference_transform with x, y, and yaw replaced.

    Z translation and the roll/pitch components of the rotation are carried over
    unchanged from reference_transform - only the planar (x, y, yaw) pose is replaced,
    since that's the only part this filter touches.
    """

    _, _, original_yaw_deg = rpy_degrees_from_rotation(reference_transform[:3, :3])
    # Rebuild rotation as roll/pitch from the original, yaw replaced - via Rodrigues on
    # a yaw-only delta rotation composed with whatever roll/pitch already existed.
    original_yaw_rad = math.radians(original_yaw_deg)
    delta_yaw = yaw_rad - original_yaw_rad
    delta_rotation, _ = cv2.Rodrigues(np.array([0.0, 0.0, delta_yaw], dtype=np.float64))

    transform = reference_transform.copy()
    transform[:3, :3] = delta_rotation @ reference_transform[:3, :3]
    transform[0, 3] = x
    transform[1, 3] = y
    return transform


class ArucoPoseMerger:
    """Merge two camera observations into marker-4 coordinates."""



    def __init__(
        self,
        *,
        marker_size_m: float = DEFAULT_MARKER_SIZE_M,
        dictionary_name: str = DEFAULT_DICTIONARY,
        target_id: int = 0,
        origin_id: int = 4,
        anchor_ids: Iterable[int] = DEFAULT_ANCHOR_IDS,
        shared_ids: Iterable[int] = DEFAULT_SHARED_IDS,
        filter_sample_rate_hz: float = 30.0,
        filter_cutoff_hz: float = 3.0,
        filter_order: int = 2,
        lock_anchors: bool = False,
    ) -> None:
        self.target_id = int(target_id)
        self.origin_id = int(origin_id)
        self.anchor_ids = tuple(int(value) for value in anchor_ids)
        self.shared_ids = tuple(int(value) for value in shared_ids)
        self.detector = ArucoPoseDetector(marker_size_m, dictionary_name)
        # Smooths the final target_in_layout / target_in_origin (x, y, yaw) - the values
        # that fluctuate the most since they combine both cameras' independent noise.
        # cutoff_hz=3.0 at a 30fps capture rate passes deliberate motion while damping
        # frame-to-frame detection jitter; lower it further if it's still too noisy, or
        # raise it if the car's motion starts to feel laggy/delayed.
        self._pose_filter = PoseButterworthFilter(
            sample_rate_hz=filter_sample_rate_hz,
            cutoff_hz=filter_cutoff_hz,
            order=filter_order,
        )

        # --- ANCHOR PERSISTENCE / LOCKING ---
        # Fixed anchor markers (and the shared markers used to register the two
        # cameras) should not vanish from the solve just because they were
        # momentarily occluded or motion-blurred out of one frame. Once an anchor
        # has been seen by a camera we cache its last-known transform and keep
        # using it until a fresher detection replaces it, the same way the
        # reference localization node latches each corner marker's corners into
        # `saved_marker_corners` instead of re-detecting all four every frame.
        # The target marker is deliberately excluded - it moves, so it must
        # always come from a live detection.
        self._cached_poses_cam1: dict[int, np.ndarray] = {}
        self._cached_poses_cam2: dict[int, np.ndarray] = {}
        self._anchors_locked = False
        self.lock_anchors = bool(lock_anchors)

    @property
    def anchors_locked(self) -> bool:
        """Return whether every anchor id has been cached by at least one camera."""

        seen_ids = set(self._cached_poses_cam1) | set(self._cached_poses_cam2)
        return self._anchors_locked or seen_ids.issuperset(self.anchor_ids)

    def _latched_poses(
        self,
        live_poses: dict[int, np.ndarray],
        cache: dict[int, np.ndarray],
    ) -> dict[int, np.ndarray]:
        """Overlay live anchor/shared-id detections onto the cached last-known poses.

        Live detections always win when available (they're more accurate), but a
        marker id that drops out of view for a frame - occlusion, motion blur,
        a bot briefly covering it - keeps using its last cached transform
        instead of disappearing from the solve. The target id is never cached:
        it moves, so it must come from a live detection or not be reported at all.
        """

        wanted_ids = set(self.anchor_ids) | set(self.shared_ids)
        
        # If lock_anchors is enabled, we only update the cache if the lock is not yet active.
        # Lock becomes active once we have cached all the required anchor markers.
        all_anchors_seen = set(self._cached_poses_cam1) | set(self._cached_poses_cam2)
        lock_active = self.lock_anchors and all_anchors_seen.issuperset(self.anchor_ids)

        if not lock_active:
            for marker_id, transform in live_poses.items():
                if marker_id in wanted_ids:
                    cache[marker_id] = transform

        latched = dict(cache)
        latched.update(
            {
                marker_id: transform
                for marker_id, transform in live_poses.items()
                if marker_id == self.target_id
            }
        )
        return latched

    def detect_pair(
        self,
        frame_cam1: np.ndarray,
        frame_cam2: np.ndarray,
        intrinsics_cam1: Intrinsics,
        intrinsics_cam2: Intrinsics,
    ) -> tuple[dict[int, MarkerPose], dict[int, MarkerPose], MergeResult]:
        """Detect and merge poses from one two-camera frame pair."""

        wanted_ids = set(self.anchor_ids) | {self.target_id}
        detections_cam1 = self.detector.detect(
            frame_cam1,
            intrinsics_cam1,
            camera_index=1,
            allowed_ids=wanted_ids,
        )
        detections_cam2 = self.detector.detect(
            frame_cam2,
            intrinsics_cam2,
            camera_index=2,
            allowed_ids=wanted_ids,
        )
        result = self.merge_detections(detections_cam1, detections_cam2)
        return detections_cam1, detections_cam2, result

    def merge_detections(
        self,
        detections_cam1: dict[int, MarkerPose],
        detections_cam2: dict[int, MarkerPose],
    ) -> MergeResult:
        """Merge already-detected marker poses."""

        poses_cam1 = {
            marker_id: pose.transform_camera_marker
            for marker_id, pose in detections_cam1.items()
        }
        poses_cam2 = {
            marker_id: pose.transform_camera_marker
            for marker_id, pose in detections_cam2.items()
        }

        # Latch anchor/shared-id poses against the persistent cache so a marker
        # that's briefly out of view doesn't drop the whole solve - see
        # `_latched_poses` for why this mirrors the reference node's
        # saved_marker_corners/calibration_locked pattern.
        poses_cam1 = self._latched_poses(poses_cam1, self._cached_poses_cam1)
        poses_cam2 = self._latched_poses(poses_cam2, self._cached_poses_cam2)

        camera2_to_camera1, used_shared_ids, residuals = self._camera2_to_camera1(
            poses_cam1,
            poses_cam2,
        )

        pose_candidates: dict[int, list[np.ndarray]] = {}
        observed_by: dict[int, list[int]] = {}
        for marker_id, transform in poses_cam1.items():
            pose_candidates.setdefault(marker_id, []).append(transform)
            observed_by.setdefault(marker_id, []).append(1)

        if camera2_to_camera1 is not None:
            for marker_id, transform in poses_cam2.items():
                transformed = camera2_to_camera1 @ transform
                pose_candidates.setdefault(marker_id, []).append(transformed)
                observed_by.setdefault(marker_id, []).append(2)

        poses_in_camera1 = {
            marker_id: average_transforms(candidates)
            for marker_id, candidates in pose_candidates.items()
        }

        poses_in_origin: dict[int, np.ndarray] = {}
        if self.origin_id in poses_in_camera1:
            camera1_to_origin = invert_transform(poses_in_camera1[self.origin_id])
            poses_in_origin = {
                marker_id: camera1_to_origin @ transform
                for marker_id, transform in poses_in_camera1.items()
            }

        target_in_origin = poses_in_origin.get(self.target_id)
        if target_in_origin is not None:
            target_in_origin = target_in_origin.copy()
            target_in_origin[:3, 3] *= 100.0
        layout_from_origin = self._layout_from_origin(poses_in_origin)
        poses_in_layout: dict[int, np.ndarray] = {}
        if layout_from_origin is not None:
            poses_in_layout = {
                marker_id: layout_from_origin @ transform
                for marker_id, transform in poses_in_origin.items()
            }
        target_in_layout = poses_in_layout.get(self.target_id)
        if target_in_layout is not None:
            target_in_layout = target_in_layout.copy()
            target_in_layout[:3, 3] *= 100.0

            x, y = float(target_in_layout[0, 3]), float(target_in_layout[1, 3])
            _, _, yaw_deg = rpy_degrees_from_rotation(target_in_layout[:3, :3])
            filtered_x, filtered_y, filtered_yaw_rad = self._pose_filter.update(
                x, y, math.radians(yaw_deg)
            )
            target_in_layout = transform_with_planar_pose(
                target_in_layout, filtered_x, filtered_y, filtered_yaw_rad
            )

        target_relative_to_anchors: dict[int, np.ndarray] = {}
        if self.target_id in poses_in_camera1:
            target_in_camera1 = poses_in_camera1[self.target_id]
            for marker_id in self.anchor_ids:
                if marker_id not in poses_in_camera1:
                    continue
                target_relative_to_anchors[marker_id] = (
                    invert_transform(poses_in_camera1[marker_id]) @ target_in_camera1
                )

        return MergeResult(
            timestamp_s=time.time(),
            used_shared_ids=tuple(used_shared_ids),
            camera2_to_camera1=camera2_to_camera1,
            poses_in_camera1=poses_in_camera1,
            poses_in_origin=poses_in_origin,
            target_in_origin=target_in_origin,
            layout_from_origin=layout_from_origin,
            poses_in_layout=poses_in_layout,
            target_in_layout=target_in_layout,
            target_relative_to_anchors=target_relative_to_anchors,
            observed_by={
                marker_id: tuple(sorted(set(camera_indices)))
                for marker_id, camera_indices in observed_by.items()
            },
            merge_residuals=residuals,
        )

    def _camera2_to_camera1(
        self,
        poses_cam1: dict[int, np.ndarray],
        poses_cam2: dict[int, np.ndarray],
    ) -> tuple[np.ndarray | None, list[int], dict[int, dict[str, float]]]:
        """Estimate camera-2 coordinates in the camera-1 frame from shared markers."""

        candidates: list[np.ndarray] = []
        used_shared_ids: list[int] = []
        for marker_id in self.shared_ids:
            if marker_id not in poses_cam1 or marker_id not in poses_cam2:
                continue
            candidates.append(
                poses_cam1[marker_id] @ invert_transform(poses_cam2[marker_id])
            )
            used_shared_ids.append(marker_id)

        if not candidates:
            return None, [], {}

        merged = average_transforms(candidates)
        residuals = {
            marker_id: transform_error(merged, candidate)
            for marker_id, candidate in zip(used_shared_ids, candidates, strict=True)
        }
        return merged, used_shared_ids, residuals

    def _layout_from_origin(
        self,
        poses_in_origin: dict[int, np.ndarray],
    ) -> np.ndarray | None:
        """Estimate the fixed floor-plan layout frame from observed anchor positions."""

        source_points: list[np.ndarray] = []
        target_points: list[np.ndarray] = []
        for marker_id, expected_xyz in LAYOUT_COORDS_M.items():
            if marker_id not in poses_in_origin:
                continue
            source_points.append(poses_in_origin[marker_id][:3, 3])
            target_points.append(np.array(expected_xyz, dtype=np.float64))

        if len(source_points) < 2:
            return None
        return planar_similarity_transform_from_points(source_points, target_points)


def find_usb_cameras() -> list[str]:
    """Return every /dev/videoN path whose hardware bus is USB.

    Detection is done purely by bus type — the videoN number is irrelevant.
    Built-in laptop webcams (ID_BUS=pci / platform), virtual devices
    (v4l2loopback), and anything with no bus attribute are all excluded.

    Two strategies are tried in order:

    1. **pyudev** (preferred) — queries the udev database for the ID_BUS
       attribute and walks the parent device chain until it finds one.
    2. **sysfs modalias fallback** — reads
       /sys/class/video4linux/videoN/device/modalias and checks for the
       ``usb:`` prefix that the USB subsystem always sets.

    Returns a sorted list, e.g. ['/dev/video2', '/dev/video6'].
    """
    import glob
    import os

    usb_devices: list[str] = []

    # ── Strategy 1: pyudev ──────────────────────────────────────────────────
    try:
        import pyudev  # type: ignore
        ctx = pyudev.Context()
        for device in ctx.list_devices(subsystem="video4linux"):
            dev_node = device.device_node
            if dev_node is None:
                continue
            parent = device
            bus: str | None = None
            while parent is not None:
                bus = parent.get("ID_BUS")
                if bus is not None:
                    break
                parent = parent.parent
            if bus == "usb":
                usb_devices.append(dev_node)
        usb_devices.sort()
        return usb_devices
    except ImportError:
        pass

    # ── Strategy 2: sysfs modalias ──────────────────────────────────────────
    for video_path in sorted(glob.glob("/dev/video*")):
        name = os.path.basename(video_path)
        modalias_path = f"/sys/class/video4linux/{name}/device/modalias"
        try:
            with open(modalias_path) as fh:
                if fh.read().strip().startswith("usb:"):
                    usb_devices.append(video_path)
        except OSError:
            pass

    return usb_devices


def require_usb_camera(path: str) -> None:
    """Raise RuntimeError if *path* is not a confirmed USB camera device.

    Called right before opening any /dev/videoN so that a wrong index or a
    system where the laptop webcam happens to have a low device number can
    never silently slip through.
    """
    usb_cams = find_usb_cameras()
    if not usb_cams:
        # Cannot confirm either way (no udev / sysfs data) — let it through
        # with a warning rather than hard-blocking on every headless system.
        import warnings
        warnings.warn(
            f"Could not verify USB bus for {path!r}; proceeding without confirmation.",
            RuntimeWarning,
            stacklevel=3,
        )
        return
    if path not in usb_cams:
        raise RuntimeError(
            f"Device {path!r} is NOT a USB camera and will not be opened.\n"
            f"USB cameras detected on this system: {usb_cams}\n"
            "Pass one of those paths, or run with --list-cameras to inspect them."
        )


class FrameSource:
    """Camera or static-image frame source."""

    def __init__(
        self,
        source: str,
        *,
        width: int | None = None,
        height: int | None = None,
        fps: float | None = None,
        fourcc: str | None = None,
        settings: dict[str, Any] | None = None,
    ) -> None:
        self.source = source
        self.static_frame: np.ndarray | None = None
        self.capture: cv2.VideoCapture | None = None

        path = Path(source)
        if path.exists() and path.is_file():
            frame = cv2.imread(str(path), cv2.IMREAD_COLOR)
            if frame is None:
                self.capture = cv2.VideoCapture(str(path))
            else:
                self.static_frame = frame
            return

        capture_source = self.resolve_capture_source(source)
        self.capture = cv2.VideoCapture(capture_source, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            raise RuntimeError(f"Could not open frame source: {source}")
        self.configure_capture(
            width=width,
            height=height,
            fps=fps,
            fourcc=fourcc,
            settings=settings,
        )

    @staticmethod
    def resolve_capture_source(source: str) -> int | str:
        """Resolve a camera source string and enforce USB-only for /dev/videoN.

        - ``/dev/videoN`` paths and bare numeric indices are validated via
          ``require_usb_camera()`` before being returned.  If the device is not
          a USB camera, ``require_usb_camera`` raises ``RuntimeError`` —
          regardless of the videoN number.
        - RTSP URLs, file paths, and other non-device strings pass through
          unchanged (they have their own validation when opened).
        """

        # Non-device sources pass through untouched.
        if not source.isdigit() and not source.startswith("/dev/video"):
            return source

        # Canonicalise to /dev/videoN.
        video_path = f"/dev/video{source}" if source.isdigit() else source

        # Hard-block anything that is not a USB camera — bus type, not path number.
        require_usb_camera(video_path)

        return video_path if Path(video_path).exists() else (int(source) if source.isdigit() else source)

    def configure_capture(
        self,
        *,
        width: int | None,
        height: int | None,
        fps: float | None,
        fourcc: str | None,
        settings: dict[str, Any] | None,
    ) -> None:
        """Set common camera properties before the first read."""

        if self.capture is None:
            return
        if fourcc:
            self.capture.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*fourcc[:4]),
            )
        if width is not None and width > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(width))
        if height is not None and height > 0:
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(height))
        if fps is not None and fps > 0.0:
            self.capture.set(cv2.CAP_PROP_FPS, float(fps))
        self.apply_settings(settings or {})

    def apply_settings(self, settings: dict[str, Any]) -> None:
        """Apply per-camera settings loaded from a YAML/JSON file."""

        if self.capture is None:
            return

        for raw_key, raw_value in settings.items():
            normalized_key = str(raw_key).strip().lower()
            if normalized_key in {"device", "ros_topic", "frame_id", "backend", "controls"}:
                continue
            if normalized_key == "fourcc_string":
                fourcc = str(raw_value).strip()
                if fourcc:
                    self.capture.set(
                        cv2.CAP_PROP_FOURCC,
                        cv2.VideoWriter_fourcc(*fourcc[:4]),
                    )
                continue
            if normalized_key == "fourcc":
                try:
                    self.capture.set(cv2.CAP_PROP_FOURCC, float(raw_value))
                except (TypeError, ValueError):
                    pass
                continue

            prop_id = CAMERA_SETTING_KEYS.get(normalized_key)
            if prop_id is None:
                continue
            try:
                value = float(raw_value)
            except (TypeError, ValueError):
                continue
            self.capture.set(prop_id, value)

    @property
    def is_static(self) -> bool:
        """Return whether this source is a still image."""

        return self.static_frame is not None

    def read(self) -> tuple[bool, np.ndarray | None]:
        """Read one frame."""

        if self.static_frame is not None:
            return True, self.static_frame.copy()
        if self.capture is None:
            return False, None
        return self.capture.read()

    def release(self) -> None:
        """Release any live camera resource."""

        if self.capture is not None:
            self.capture.release()


def draw_detections(
    frame: np.ndarray,
    detections: dict[int, MarkerPose],
    intrinsics: Intrinsics,
    label: str,
    axis_length_m: float,
) -> np.ndarray:
    """Draw detected marker borders and axes."""

    preview = frame.copy()
    if detections:
        corners = [
            pose.corners.reshape(1, 4, 2).astype(np.float32)
            for pose in detections.values()
        ]
        ids = np.array([[marker_id] for marker_id in detections], dtype=np.int32)
        cv2.aruco.drawDetectedMarkers(preview, corners, ids)
        for pose in detections.values():
            cv2.drawFrameAxes(
                preview,
                intrinsics.camera_matrix,
                intrinsics.distortion,
                pose.rvec,
                pose.tvec,
                axis_length_m,
            )

    cv2.putText(
        preview,
        label,
        (20, 36),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return preview


def _resize_to_height(frame: np.ndarray, target_height: int) -> np.ndarray:
    """Resize a frame to a common height while preserving aspect ratio."""

    height, width = frame.shape[:2]
    if height <= 0 or width <= 0 or height == target_height:
        return frame
    scale = target_height / float(height)
    target_width = max(1, int(round(width * scale)))
    return cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)


def resize_for_display(
    image: np.ndarray,
    *,
    max_width: int = 1280,
    max_height: int = 900,
) -> np.ndarray:
    """Shrink a preview image to fit on screen while preserving aspect ratio."""

    height, width = image.shape[:2]
    if height <= 0 or width <= 0:
        return image

    scale = min(max_width / float(width), max_height / float(height), 1.0)
    if scale >= 1.0:
        return image

    target_width = max(1, int(round(width * scale)))
    target_height = max(1, int(round(height * scale)))
    return cv2.resize(
        image,
        (target_width, target_height),
        interpolation=cv2.INTER_AREA,
    )


def _draw_status_lines(
    image: np.ndarray,
    lines: Sequence[str],
    *,
    origin: tuple[int, int],
    line_height: int = 26,
    color: tuple[int, int, int] = (235, 235, 235),
) -> None:
    """Render a block of readable debug text."""

    x, y = origin
    for index, line in enumerate(lines):
        y_line = y + index * line_height
        cv2.putText(
            image,
            line,
            (x, y_line),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (20, 20, 20),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            image,
            line,
            (x, y_line),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            1,
            cv2.LINE_AA,
        )


def _draw_highlight_line(
    image: np.ndarray,
    text: str,
    *,
    origin: tuple[int, int],
    font_scale: float = 0.95,
    color: tuple[int, int, int] = (255, 255, 255),
    accent_color: tuple[int, int, int] = (40, 180, 255),
) -> None:
    """Render a larger highlighted status line."""

    x, y = origin
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (15, 15, 15),
        6,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        accent_color,
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        image,
        text,
        (x, y),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        color,
        1,
        cv2.LINE_AA,
    )


def draw_merged_debug_view(
    preview_cam1: np.ndarray,
    preview_cam2: np.ndarray,
    result: MergeResult,
    *,
    topic_cam1: str,
    topic_cam2: str,
    target_id: int,
    origin_id: int,
) -> np.ndarray:
    """Compose a live stacked debug view with merge status."""

    common_width = max(preview_cam1.shape[1], preview_cam2.shape[1])
    view_cam1 = preview_cam1
    view_cam2 = preview_cam2
    if preview_cam1.shape[1] != common_width:
        target_height = max(
            1,
            int(round(preview_cam1.shape[0] * (common_width / float(preview_cam1.shape[1])))),
        )
        view_cam1 = cv2.resize(
            preview_cam1,
            (common_width, target_height),
            interpolation=cv2.INTER_AREA,
        )
    if preview_cam2.shape[1] != common_width:
        target_height = max(
            1,
            int(round(preview_cam2.shape[0] * (common_width / float(preview_cam2.shape[1])))),
        )
        view_cam2 = cv2.resize(
            preview_cam2,
            (common_width, target_height),
            interpolation=cv2.INTER_AREA,
        )

    gap = 12
    status_height = 170
    stack_height = view_cam2.shape[0] + view_cam1.shape[0] + gap * 3
    canvas_width = common_width + gap * 2
    canvas_height = stack_height + status_height + gap * 2
    canvas = np.full((canvas_height, canvas_width, 3), 24, dtype=np.uint8)

    # camera1 on TOP, camera2 on BOTTOM
    y_cam1 = gap
    canvas[y_cam1 : y_cam1 + view_cam1.shape[0], gap : gap + view_cam1.shape[1]] = view_cam1
    y_cam2 = y_cam1 + view_cam1.shape[0] + gap
    canvas[y_cam2 : y_cam2 + view_cam2.shape[0], gap : gap + view_cam2.shape[1]] = view_cam2

    cv2.line(
        canvas,
        (gap, y_cam2 - gap // 2),
        (gap + common_width, y_cam2 - gap // 2),
        (70, 70, 70),
        2,
        cv2.LINE_AA,
    )

    status_lines = [
        f"cam1 topic: {topic_cam1}",
        f"cam2 topic: {topic_cam2}",
        f"cam1 expects ids {list(EXPECTED_IDS_CAM1)} | cam2 expects ids {list(EXPECTED_IDS_CAM2)}",
        "layout origin: id4 -> (0.0, 0.0)",
        f"shared ids: {list(result.used_shared_ids) or 'none'}",
        f"origin id{origin_id}: {'visible' if origin_id in result.poses_in_origin else 'missing'}",
        (
            "camera2->camera1: ready"
            if result.camera2_to_camera1 is not None
            else "camera2->camera1: unavailable"
        ),
    ]
    highlight_line = f"id{target_id} in layout: unavailable"
    if result.target_in_layout is not None:
        translation = result.target_in_layout[:3, 3]
        _roll_deg, _pitch_deg, yaw_deg = rpy_degrees_from_rotation(
            result.target_in_layout[:3, :3]
        )
        highlight_line = (
            f"id{target_id}: x={translation[0]:+.3f} y={translation[1]:+.3f} "
            f"z={translation[2]:+.3f} yaw={yaw_deg:+.1f} deg"
        )
    elif result.target_in_origin is not None:
        translation = result.target_in_origin[:3, 3]
        _roll_deg, _pitch_deg, yaw_deg = rpy_degrees_from_rotation(
            result.target_in_origin[:3, :3]
        )
        highlight_line = (
            f"id{target_id}: x={translation[0]:+.3f} y={translation[1]:+.3f} "
            f"z={translation[2]:+.3f} yaw={yaw_deg:+.1f} deg"
        )
    status_lines.append(highlight_line)

    if result.merge_residuals:
        residual_text = ", ".join(
            f"id{marker_id}={values['translation_m'] * 1000.0:.1f}mm/{values['rotation_deg']:.2f}deg"
            for marker_id, values in sorted(result.merge_residuals.items())
        )
        status_lines.append(f"shared residuals: {residual_text}")
    else:
        status_lines.append("shared residuals: none")

    _draw_status_lines(
        canvas,
        ["merged debug view"] + list(status_lines),
        origin=(gap, stack_height + 58),
    )
    _draw_highlight_line(
        canvas,
        highlight_line,
        origin=(gap, stack_height + 24),
    )
    return canvas


def result_to_payload(
    result: MergeResult,
    *,
    target_id: int,
    origin_id: int,
) -> dict[str, Any]:
    """Convert a merge result into JSON-friendly data."""

    payload: dict[str, Any] = {
        "timestamp_s": result.timestamp_s,
        "target_id": target_id,
        "origin_id": origin_id,
        "used_shared_ids": list(result.used_shared_ids),
        "observed_by": {
            str(marker_id): list(camera_indices)
            for marker_id, camera_indices in result.observed_by.items()
        },
        "merge_residuals": {
            str(marker_id): values
            for marker_id, values in result.merge_residuals.items()
        },
        "target_in_origin": (
            None if result.target_in_origin is None else pose_payload(result.target_in_origin)
        ),
        "target_in_layout": (
            None if result.target_in_layout is None else pose_payload(result.target_in_layout)
        ),
        "anchors_in_origin": {
            str(marker_id): pose_payload(transform)
            for marker_id, transform in sorted(result.poses_in_origin.items())
            if marker_id != target_id
        },
        "anchors_in_layout": {
            str(marker_id): pose_payload(transform)
            for marker_id, transform in sorted(result.poses_in_layout.items())
            if marker_id != target_id
        },
        "target_relative_to_anchors": {
            str(marker_id): pose_payload(transform)
            for marker_id, transform in sorted(result.target_relative_to_anchors.items())
        },
        "layout_coords_m": {
            str(marker_id): list(coords)
            for marker_id, coords in sorted(LAYOUT_COORDS_M.items())
        },
    }
    if result.camera2_to_camera1 is not None:
        payload["camera2_to_camera1"] = pose_payload(result.camera2_to_camera1)
    else:
        payload["camera2_to_camera1"] = None
    if result.layout_from_origin is not None:
        payload["layout_from_origin"] = pose_payload(result.layout_from_origin)
    else:
        payload["layout_from_origin"] = None
    return payload


def format_pose_line(prefix: str, transform: np.ndarray) -> str:
    """Return a compact human-readable pose summary."""

    translation = transform[:3, 3]
    roll_deg, pitch_deg, yaw_deg = rpy_degrees_from_rotation(transform[:3, :3])
    return (
        f"{prefix}: x={translation[0]:+.3f} m, y={translation[1]:+.3f} m, "
        f"z={translation[2]:+.3f} m, yaw={yaw_deg:+.1f} deg, "
        f"roll={roll_deg:+.1f} deg, pitch={pitch_deg:+.1f} deg"
    )


def print_human_result(
    result: MergeResult,
    *,
    target_id: int,
    origin_id: int,
    anchor_ids: Sequence[int],
) -> None:
    """Print a concise human-readable merge result."""

    observed = ", ".join(
        f"{marker_id}:cam{''.join(str(index) for index in cameras)}"
        for marker_id, cameras in sorted(result.observed_by.items())
    )
    print(f"observed [{observed or 'none'}], shared={list(result.used_shared_ids)}")

    if result.camera2_to_camera1 is None:
        print("camera2_to_camera1: unavailable; need shared IDs in both cameras")
    else:
        print(format_pose_line("camera2_to_camera1", result.camera2_to_camera1))
        if result.merge_residuals:
            residual = ", ".join(
                (
                    f"id{marker_id}: {values['translation_m'] * 1000.0:.1f} mm, "
                    f"{values['rotation_deg']:.2f} deg"
                )
                for marker_id, values in sorted(result.merge_residuals.items())
            )
            print(f"shared-marker residuals: {residual}")

    if origin_id not in result.poses_in_origin:
        print(f"origin marker {origin_id}: unavailable")
        return

    if result.poses_in_layout:
        for marker_id in anchor_ids:
            if marker_id not in result.poses_in_layout:
                continue
            print(format_pose_line(f"id{marker_id} in layout", result.poses_in_layout[marker_id]))
    else:
        for marker_id in anchor_ids:
            if marker_id not in result.poses_in_origin:
                continue
            print(format_pose_line(f"id{marker_id} in id{origin_id}", result.poses_in_origin[marker_id]))

    if result.target_in_layout is not None:
        print(format_pose_line(f"id{target_id} in layout", result.target_in_layout))
        return

    if result.target_in_origin is None:
        print(f"target marker {target_id}: unavailable")
        return

    print(format_pose_line(f"id{target_id} in id{origin_id}", result.target_in_origin))


def maybe_write_json(path: Path | None, payload: dict[str, Any]) -> None:
    """Write JSON output if a path was requested."""

    if path is None:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2), encoding="utf-8")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse CLI arguments."""

    parser = argparse.ArgumentParser(
        description=(
            "Merge two ArUco camera views. Camera 1 should see IDs 1,2,3,4; "
            "camera 2 should see IDs 3,4,5,6; shared IDs 3,4 estimate camera2 "
            "in camera1 coordinates; marker 0 is reported in marker-4 coordinates."
        )
    )
    parser.add_argument("--cam1", default=None, help="Camera-1 full path, stream URL, or video file. Overrides --cam1-index.")
    parser.add_argument("--cam2", default=None, help="Camera-2 full path, stream URL, or video file. Overrides --cam2-index.")
    parser.add_argument(
        "--cam1-index",
        type=int,
        default=None,
        metavar="N",
        help="Set the N in /dev/videoN for camera 1 (e.g. --cam1-index 2 → /dev/video2). "
             "Ignored when --cam1 is also set.",
    )
    parser.add_argument(
        "--cam2-index",
        type=int,
        default=None,
        metavar="N",
        help="Set the N in /dev/videoN for camera 2 (e.g. --cam2-index 4 → /dev/video4). "
             "Ignored when --cam2 is also set.",
    )
    parser.add_argument(
        "--intrinsics1",
        type=Path,
        default=DEFAULT_INTRINSICS_CAM1,
        help="Camera-1 intrinsics YAML/JSON.",
    )
    parser.add_argument(
        "--intrinsics2",
        type=Path,
        default=DEFAULT_INTRINSICS_CAM2,
        help="Camera-2 intrinsics YAML/JSON.",
    )
    parser.add_argument(
        "--settings1",
        type=Path,
        default=DEFAULT_SETTINGS_CAM1,
        help="Camera-1 settings YAML/JSON.",
    )
    parser.add_argument(
        "--settings2",
        type=Path,
        default=DEFAULT_SETTINGS_CAM2,
        help="Camera-2 settings YAML/JSON.",
    )
    parser.add_argument(
        "--dictionary",
        default=DEFAULT_DICTIONARY,
        help="OpenCV ArUco dictionary constant, e.g. DICT_4X4_50.",
    )
    parser.add_argument(
        "--marker-size-m",
        type=float,
        default=DEFAULT_MARKER_SIZE_M,
        help="Marker side length in metres. Default: 0.20.",
    )
    parser.add_argument("--target-id", type=int, default=0, help="Marker id to report.")
    parser.add_argument("--origin-id", type=int, default=4, help="Marker id used as origin.")
    parser.add_argument(
        "--anchor-ids",
        default="1,2,3,4,5,6",
        help="Comma-separated fixed anchor marker IDs.",
    )
    parser.add_argument(
        "--shared-ids",
        default="3,4",
        help="Comma-separated marker IDs visible to both cameras.",
    )
    parser.add_argument("--once", action="store_true", help="Process one frame pair and exit.")
    parser.add_argument("--json", action="store_true", help="Print JSON instead of human-readable text.")
    parser.add_argument("--output-json", type=Path, default=None, help="Write the latest result to this JSON file.")
    parser.add_argument("--no-preview", dest="preview", action="store_false", help="Disable the OpenCV stacked preview window.")
    parser.set_defaults(preview=True)
    parser.add_argument("--print-hz", type=float, default=5.0, help="Print frequency for live streams.")
    parser.add_argument("--warmup-frames", type=int, default=0, help="Discard this many initial frames.")
    parser.add_argument(
        "--frame-width",
        type=int,
        default=640,
        help="Requested camera frame width.",
    )
    parser.add_argument(
        "--frame-height",
        type=int,
        default=360,
        help="Requested camera frame height.",
    )
    parser.add_argument("--fps", type=float, default=30.0, help="Requested camera FPS.")
    parser.add_argument(
        "--filter-cutoff-hz",
        type=float,
        default=3.0,
        help=(
            "Butterworth low-pass cutoff frequency (Hz) applied to the reported "
            "target x, y, yaw. Lower = smoother but more lag; higher = more "
            "responsive but noisier. Must stay below fps/2."
        ),
    )
    parser.add_argument("--fourcc", default="MJPG", help="Requested V4L2 pixel format.")
    parser.add_argument(
        "--lock-anchors",
        action="store_true",
        help="Lock/freeze anchor marker poses once all anchors have been seen to eliminate camera-calibration jitter.",
    )
    parser.add_argument(
        "--list-cameras",
        action="store_true",
        help="Print all detected USB cameras and exit.",
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the CLI."""

    args = parse_args(argv)

    # --list-cameras: enumerate USB devices and exit.
    if args.list_cameras:
        usb_cams = find_usb_cameras()
        if usb_cams:
            print("Detected USB cameras:")
            for cam in usb_cams:
                print(f"  {cam}")
        else:
            print("No USB cameras detected.")
        return 0

    # --- Resolve camera sources (USB cameras only) -------------------------
    # Auto-discovery always runs first so we know what is physically available.
    # The videoN number does NOT matter — only the USB bus type does.
    usb_cams = find_usb_cameras()
    if not usb_cams:
        print("WARNING: No USB cameras detected on this system. Proceeding anyway.")

    def pick_usb_cam(index: int) -> str:
        """Return the Nth (0-based) detected USB camera, or abort if not enough."""
        if len(usb_cams) > index:
            return usb_cams[index]
        raise RuntimeError(
            f"Need at least {index + 1} USB camera(s) but only found {len(usb_cams)}: {usb_cams}"
        )

    # cam1
    if args.cam1 is not None:
        # Explicit path supplied — still enforce USB if it looks like a device.
        if args.cam1.startswith("/dev/video"):
            require_usb_camera(args.cam1)
            print(f"cam1 → {args.cam1}  (explicit path, confirmed USB)")
    else:
        args.cam1 = pick_usb_cam(0)
        print(f"cam1 → {args.cam1}  (auto-selected USB camera 1 of {len(usb_cams)})")

    # cam2
    if args.cam2 is not None:
        if args.cam2.startswith("/dev/video"):
            require_usb_camera(args.cam2)
            print(f"cam2 → {args.cam2}  (explicit path, confirmed USB)")
    else:
        args.cam2 = pick_usb_cam(1)
        print(f"cam2 → {args.cam2}  (auto-selected USB camera 2 of {len(usb_cams)})")

    intrinsics1 = load_intrinsics(args.intrinsics1)
    intrinsics2 = load_intrinsics(args.intrinsics2 or args.intrinsics1)
    settings1 = load_camera_settings(args.settings1)
    settings2 = load_camera_settings(args.settings2)
    topic1, topic2 = validate_camera_settings_pair(settings1, settings2)

    anchor_ids = parse_id_list(args.anchor_ids)
    shared_ids = parse_id_list(args.shared_ids)
    merger = ArucoPoseMerger(
        marker_size_m=args.marker_size_m,
        dictionary_name=args.dictionary,
        target_id=args.target_id,
        origin_id=args.origin_id,
        anchor_ids=anchor_ids,
        shared_ids=shared_ids,
        filter_sample_rate_hz=args.fps,
        filter_cutoff_hz=args.filter_cutoff_hz,
        lock_anchors=args.lock_anchors,
    )

    source1 = FrameSource(
        args.cam1,
        width=args.frame_width,
        height=args.frame_height,
        fps=args.fps,
        fourcc=args.fourcc,
        settings=settings1,
    )
    source2 = FrameSource(
        args.cam2,
        width=args.frame_width,
        height=args.frame_height,
        fps=args.fps,
        fourcc=args.fourcc,
        settings=settings2,
    )
    next_print_s = 0.0

    try:
        for _ in range(max(0, args.warmup_frames)):
            source1.read()
            source2.read()

        while True:
            ok1, frame1 = source1.read()
            ok2, frame2 = source2.read()
            if not ok1 or frame1 is None:
                raise RuntimeError(f"Failed to read frame from camera 1 source {args.cam1!r}.")
            if not ok2 or frame2 is None:
                raise RuntimeError(f"Failed to read frame from camera 2 source {args.cam2!r}.")

            frame_intrinsics1 = intrinsics1.scaled_to_frame(frame1)
            frame_intrinsics2 = intrinsics2.scaled_to_frame(frame2)
            detections1, detections2, result = merger.detect_pair(
                frame1,
                frame2,
                frame_intrinsics1,
                frame_intrinsics2,
            )
            payload = result_to_payload(
                result,
                target_id=args.target_id,
                origin_id=args.origin_id,
            )
            maybe_write_json(args.output_json, payload)

            now_s = time.time()
            should_print = args.once or now_s >= next_print_s
            if should_print:
                if args.json:
                    indent = 2 if args.once else None
                    print(json.dumps(payload, indent=indent), flush=True)
                else:
                    print_human_result(
                        result,
                        target_id=args.target_id,
                        origin_id=args.origin_id,
                        anchor_ids=anchor_ids,
                    )
                    print("", flush=True)
                interval_s = 1.0 / max(0.1, args.print_hz)
                next_print_s = now_s + interval_s

            if args.preview:
                axis_length = args.marker_size_m * 0.5
                preview1 = draw_detections(
                    frame1,
                    detections1,
                    frame_intrinsics1,
                    "camera1",
                    axis_length,
                )
                preview2 = draw_detections(
                    frame2,
                    detections2,
                    frame_intrinsics2,
                    "camera2",
                    axis_length,
                )
                cv2.imshow(
                    "monocular_merge merged",
                    resize_for_display(
                        draw_merged_debug_view(
                            preview1,
                            preview2,
                            result,
                            topic_cam1=topic1,
                            topic_cam2=topic2,
                            target_id=args.target_id,
                            origin_id=args.origin_id,
                        )
                    ),
                )
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            if args.once or (source1.is_static and source2.is_static):
                break
    finally:
        source1.release()
        source2.release()
        if args.preview:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())