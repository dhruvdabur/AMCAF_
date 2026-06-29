#!/usr/bin/env python3
"""Merge two monocular ArUco views and report marker 0 in marker 1 coordinates.

OpenCV returns each marker pose as ``T_camera_marker``: points expressed in the
marker frame are transformed into the camera frame. This module uses shared
anchor markers to estimate ``T_camera0_camera1``, then expresses every detected
marker in the marker-1 frame.
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


DEFAULT_INTRINSICS = Path("camera_calibration/camera_intrinsics.yaml")
DEFAULT_MARKER_SIZE_M = 0.20
DEFAULT_DICTIONARY = "DICT_4X4_50"
DEFAULT_ANCHOR_IDS = (1, 2, 3, 4, 5, 6)
DEFAULT_SHARED_IDS = (3, 4)


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
    camera1_to_camera0: np.ndarray | None
    poses_in_camera0: dict[int, np.ndarray]
    poses_in_origin: dict[int, np.ndarray]
    target_in_origin: np.ndarray | None
    target_relative_to_anchors: dict[int, np.ndarray]
    observed_by: dict[int, tuple[int, ...]]
    merge_residuals: dict[int, dict[str, float]]

    @property
    def has_target(self) -> bool:
        """Return whether the target marker pose was estimated."""

        return self.target_in_origin is not None


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
            ok, rvec, tvec = self._solve_marker_pose(image_points, intrinsics)
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
    ) -> tuple[bool, np.ndarray, np.ndarray]:
        """Solve one square marker pose, preferring IPPE_SQUARE when available."""

        flags = getattr(cv2, "SOLVEPNP_IPPE_SQUARE", cv2.SOLVEPNP_ITERATIVE)
        ok, rvec, tvec = cv2.solvePnP(
            self.object_points,
            image_points,
            intrinsics.camera_matrix,
            intrinsics.distortion,
            flags=flags,
        )
        if ok:
            return ok, rvec, tvec

        return cv2.solvePnP(
            self.object_points,
            image_points,
            intrinsics.camera_matrix,
            intrinsics.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )


class ArucoPoseMerger:
    """Merge two camera observations into marker-1 coordinates."""

    def __init__(
        self,
        *,
        marker_size_m: float = DEFAULT_MARKER_SIZE_M,
        dictionary_name: str = DEFAULT_DICTIONARY,
        target_id: int = 0,
        origin_id: int = 1,
        anchor_ids: Iterable[int] = DEFAULT_ANCHOR_IDS,
        shared_ids: Iterable[int] = DEFAULT_SHARED_IDS,
    ) -> None:
        self.target_id = int(target_id)
        self.origin_id = int(origin_id)
        self.anchor_ids = tuple(int(value) for value in anchor_ids)
        self.shared_ids = tuple(int(value) for value in shared_ids)
        self.detector = ArucoPoseDetector(marker_size_m, dictionary_name)

    def detect_pair(
        self,
        frame0: np.ndarray,
        frame1: np.ndarray,
        intrinsics0: Intrinsics,
        intrinsics1: Intrinsics,
    ) -> tuple[dict[int, MarkerPose], dict[int, MarkerPose], MergeResult]:
        """Detect and merge poses from one two-camera frame pair."""

        wanted_ids = set(self.anchor_ids) | {self.target_id}
        detections0 = self.detector.detect(
            frame0,
            intrinsics0,
            camera_index=0,
            allowed_ids=wanted_ids,
        )
        detections1 = self.detector.detect(
            frame1,
            intrinsics1,
            camera_index=1,
            allowed_ids=wanted_ids,
        )
        result = self.merge_detections(detections0, detections1)
        return detections0, detections1, result

    def merge_detections(
        self,
        detections0: dict[int, MarkerPose],
        detections1: dict[int, MarkerPose],
    ) -> MergeResult:
        """Merge already-detected marker poses."""

        poses0 = {
            marker_id: pose.transform_camera_marker
            for marker_id, pose in detections0.items()
        }
        poses1 = {
            marker_id: pose.transform_camera_marker
            for marker_id, pose in detections1.items()
        }

        camera1_to_camera0, used_shared_ids, residuals = self._camera1_to_camera0(
            poses0,
            poses1,
        )

        pose_candidates: dict[int, list[np.ndarray]] = {}
        observed_by: dict[int, list[int]] = {}
        for marker_id, transform in poses0.items():
            pose_candidates.setdefault(marker_id, []).append(transform)
            observed_by.setdefault(marker_id, []).append(0)

        if camera1_to_camera0 is not None:
            for marker_id, transform in poses1.items():
                transformed = camera1_to_camera0 @ transform
                pose_candidates.setdefault(marker_id, []).append(transformed)
                observed_by.setdefault(marker_id, []).append(1)

        poses_in_camera0 = {
            marker_id: average_transforms(candidates)
            for marker_id, candidates in pose_candidates.items()
        }

        poses_in_origin: dict[int, np.ndarray] = {}
        if self.origin_id in poses_in_camera0:
            camera0_to_origin = invert_transform(poses_in_camera0[self.origin_id])
            poses_in_origin = {
                marker_id: camera0_to_origin @ transform
                for marker_id, transform in poses_in_camera0.items()
            }

        target_in_origin = poses_in_origin.get(self.target_id)

        target_relative_to_anchors: dict[int, np.ndarray] = {}
        if self.target_id in poses_in_camera0:
            target_in_camera0 = poses_in_camera0[self.target_id]
            for marker_id in self.anchor_ids:
                if marker_id not in poses_in_camera0:
                    continue
                target_relative_to_anchors[marker_id] = (
                    invert_transform(poses_in_camera0[marker_id]) @ target_in_camera0
                )

        return MergeResult(
            timestamp_s=time.time(),
            used_shared_ids=tuple(used_shared_ids),
            camera1_to_camera0=camera1_to_camera0,
            poses_in_camera0=poses_in_camera0,
            poses_in_origin=poses_in_origin,
            target_in_origin=target_in_origin,
            target_relative_to_anchors=target_relative_to_anchors,
            observed_by={
                marker_id: tuple(sorted(set(camera_indices)))
                for marker_id, camera_indices in observed_by.items()
            },
            merge_residuals=residuals,
        )

    def _camera1_to_camera0(
        self,
        poses0: dict[int, np.ndarray],
        poses1: dict[int, np.ndarray],
    ) -> tuple[np.ndarray | None, list[int], dict[int, dict[str, float]]]:
        """Estimate camera-1 coordinates in the camera-0 frame from shared markers."""

        candidates: list[np.ndarray] = []
        used_shared_ids: list[int] = []
        for marker_id in self.shared_ids:
            if marker_id not in poses0 or marker_id not in poses1:
                continue
            candidates.append(poses0[marker_id] @ invert_transform(poses1[marker_id]))
            used_shared_ids.append(marker_id)

        if not candidates:
            return None, [], {}

        merged = average_transforms(candidates)
        residuals = {
            marker_id: transform_error(merged, candidate)
            for marker_id, candidate in zip(used_shared_ids, candidates, strict=True)
        }
        return merged, used_shared_ids, residuals


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
        self.configure_capture(width=width, height=height, fps=fps, fourcc=fourcc)

    @staticmethod
    def resolve_capture_source(source: str) -> int | str:
        """Prefer explicit V4L2 paths for numeric camera IDs when present."""

        if not source.isdigit():
            return source
        video_path = Path(f"/dev/video{source}")
        if video_path.exists():
            return str(video_path)
        return int(source)

    def configure_capture(
        self,
        *,
        width: int | None,
        height: int | None,
        fps: float | None,
        fourcc: str | None,
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
        "anchors_in_origin": {
            str(marker_id): pose_payload(transform)
            for marker_id, transform in sorted(result.poses_in_origin.items())
            if marker_id != target_id
        },
        "target_relative_to_anchors": {
            str(marker_id): pose_payload(transform)
            for marker_id, transform in sorted(result.target_relative_to_anchors.items())
        },
    }
    if result.camera1_to_camera0 is not None:
        payload["camera1_to_camera0"] = pose_payload(result.camera1_to_camera0)
    else:
        payload["camera1_to_camera0"] = None
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

    if result.camera1_to_camera0 is None:
        print("camera1_to_camera0: unavailable; need shared IDs in both cameras")
    else:
        print(format_pose_line("camera1_to_camera0", result.camera1_to_camera0))
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

    for marker_id in anchor_ids:
        if marker_id not in result.poses_in_origin:
            continue
        print(format_pose_line(f"id{marker_id} in id{origin_id}", result.poses_in_origin[marker_id]))

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
            "Merge two ArUco camera views. Camera 0 should see IDs 1,2,3,4; "
            "camera 1 should see IDs 3,4,5,6; marker 0 is reported in marker-1 coordinates."
        )
    )
    parser.add_argument("--cam0", default="/dev/video0", help="Camera-0 index, stream URL, video, or image path.")
    parser.add_argument("--cam1", default="/dev/video4", help="Camera-1 index, stream URL, video, or image path.")
    parser.add_argument(
        "--intrinsics0",
        type=Path,
        default=DEFAULT_INTRINSICS,
        help="Camera-0 intrinsics YAML/JSON.",
    )
    parser.add_argument(
        "--intrinsics1",
        type=Path,
        default=None,
        help="Camera-1 intrinsics YAML/JSON. Defaults to --intrinsics0.",
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
    parser.add_argument("--origin-id", type=int, default=1, help="Marker id used as origin.")
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
    parser.add_argument("--preview", action="store_true", help="Show OpenCV preview windows.")
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
    parser.add_argument("--fourcc", default="MJPG", help="Requested V4L2 pixel format.")
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Run the CLI."""

    args = parse_args(argv)
    intrinsics0 = load_intrinsics(args.intrinsics0)
    intrinsics1 = load_intrinsics(args.intrinsics1 or args.intrinsics0)

    anchor_ids = parse_id_list(args.anchor_ids)
    shared_ids = parse_id_list(args.shared_ids)
    merger = ArucoPoseMerger(
        marker_size_m=args.marker_size_m,
        dictionary_name=args.dictionary,
        target_id=args.target_id,
        origin_id=args.origin_id,
        anchor_ids=anchor_ids,
        shared_ids=shared_ids,
    )

    source0 = FrameSource(
        args.cam0,
        width=args.frame_width,
        height=args.frame_height,
        fps=args.fps,
        fourcc=args.fourcc,
    )
    source1 = FrameSource(
        args.cam1,
        width=args.frame_width,
        height=args.frame_height,
        fps=args.fps,
        fourcc=args.fourcc,
    )
    next_print_s = 0.0

    try:
        for _ in range(max(0, args.warmup_frames)):
            source0.read()
            source1.read()

        while True:
            ok0, frame0 = source0.read()
            ok1, frame1 = source1.read()
            if not ok0 or frame0 is None:
                raise RuntimeError(f"Failed to read frame from camera 0 source {args.cam0!r}.")
            if not ok1 or frame1 is None:
                raise RuntimeError(f"Failed to read frame from camera 1 source {args.cam1!r}.")

            frame_intrinsics0 = intrinsics0.scaled_to_frame(frame0)
            frame_intrinsics1 = intrinsics1.scaled_to_frame(frame1)
            detections0, detections1, result = merger.detect_pair(
                frame0,
                frame1,
                frame_intrinsics0,
                frame_intrinsics1,
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
                cv2.imshow(
                    "monocular_merge camera0",
                    draw_detections(frame0, detections0, frame_intrinsics0, "camera0", axis_length),
                )
                cv2.imshow(
                    "monocular_merge camera1",
                    draw_detections(frame1, detections1, frame_intrinsics1, "camera1", axis_length),
                )
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break

            if args.once or (source0.is_static and source1.is_static):
                break
    finally:
        source0.release()
        source1.release()
        if args.preview:
            cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
