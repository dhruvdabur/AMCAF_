#!/usr/bin/env python3
"""Interactively tune camera extrinsic pose with a live projection overlay."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


WINDOW_NAME = 'Camera Extrinsics Tuner'
CONTROL_WINDOW = 'Extrinsic Controls'
CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)


def parse_args():
    """Read camera, intrinsic, target, and output settings."""
    parser = argparse.ArgumentParser(
        description=(
            'Open a live camera preview with sliders for camera extrinsics. '
            'The saved transform maps world/object points into camera frame.'
        )
    )
    parser.add_argument(
        '--device',
        type=int,
        default=2,
        help='Video device index (default: 2, Lenovo FHD webcam).',
    )
    parser.add_argument(
        '--width',
        type=int,
        default=1280,
        help='Requested capture width (default: 1280).',
    )
    parser.add_argument(
        '--height',
        type=int,
        default=720,
        help='Requested capture height (default: 720).',
    )
    parser.add_argument(
        '--fps',
        type=float,
        default=30.0,
        help='Requested frames per second (default: 30).',
    )
    parser.add_argument(
        '--format',
        choices=('MJPG', 'YUYV'),
        default='MJPG',
        help='Requested pixel format (default: MJPG for USB bandwidth).',
    )
    parser.add_argument(
        '--intrinsics',
        type=Path,
        help='Load camera_matrix and distortion_coefficients from JSON/OpenCV YAML.',
    )
    parser.add_argument(
        '--load',
        type=Path,
        help='Load initial extrinsics from JSON or OpenCV YAML.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('camera_extrinsics.yaml'),
        help='Path to save tuned extrinsics with S (default: camera_extrinsics.yaml).',
    )
    parser.add_argument(
        '--columns',
        type=int,
        default=9,
        help='Inner chessboard corners across columns for PnP snap (default: 9).',
    )
    parser.add_argument(
        '--rows',
        type=int,
        default=6,
        help='Inner chessboard corners across rows for PnP snap (default: 6).',
    )
    parser.add_argument(
        '--square-size',
        type=float,
        default=25.0,
        help='World units per chessboard square/grid step (default: 25.0).',
    )
    parser.add_argument(
        '--grid-columns',
        type=int,
        default=10,
        help='Projected world-grid columns (default: 10).',
    )
    parser.add_argument(
        '--grid-rows',
        type=int,
        default=8,
        help='Projected world-grid rows (default: 8).',
    )
    parser.add_argument(
        '--translation-range',
        type=int,
        default=5000,
        help='Slider range for tx/ty/tz in world units (default: 5000).',
    )
    parser.add_argument(
        '--translation-scale',
        type=int,
        default=10,
        help='World units per translation slider tick (default: 10).',
    )
    return parser.parse_args()


def open_camera(args):
    """Open and configure the selected V4L2 camera."""
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'Camera /dev/video{args.device} could not be opened.')

    if args.format:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.format))
    if args.width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps:
        cap.set(cv2.CAP_PROP_FPS, args.fps)
    return cap


def default_intrinsics(width, height):
    """Return a simple pinhole guess when no intrinsics file is provided."""
    focal = float(max(width, height))
    camera_matrix = np.array(
        [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros((5, 1), dtype=np.float64)
    return camera_matrix, distortion


def load_intrinsics(path, width, height):
    """Load camera intrinsics from JSON or OpenCV YAML."""
    if path is None:
        return default_intrinsics(width, height)

    if path.suffix.lower() == '.json':
        with path.open('r', encoding='utf-8') as stream:
            payload = json.load(stream)
        camera_matrix = np.asarray(payload['camera_matrix'], dtype=np.float64)
        distortion = np.asarray(
            payload.get('distortion_coefficients', payload.get('dist_coeffs')),
            dtype=np.float64,
        ).reshape(-1, 1)
        return camera_matrix, normalize_distortion(distortion)

    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise SystemExit(f'Unable to open intrinsics file: {path}')
    try:
        camera_matrix = storage.getNode('camera_matrix').mat()
        distortion = storage.getNode('distortion_coefficients').mat()
    finally:
        storage.release()

    if camera_matrix is None or distortion is None:
        raise SystemExit(
            f'{path} must contain camera_matrix and distortion_coefficients.'
        )
    return camera_matrix.astype(np.float64), normalize_distortion(distortion)


def normalize_distortion(distortion):
    """Keep the first five OpenCV distortion coefficients as a column vector."""
    values = np.zeros((5, 1), dtype=np.float64)
    flattened = np.asarray(distortion, dtype=np.float64).reshape(-1)
    values[: min(5, flattened.size), 0] = flattened[:5]
    return values


def default_extrinsics():
    """Return a visible default pose: world plane in front of the camera."""
    rvec = np.zeros((3, 1), dtype=np.float64)
    tvec = np.array([[0.0], [0.0], [1000.0]], dtype=np.float64)
    return rvec, tvec


def load_extrinsics(path):
    """Load rvec/tvec or transform_matrix from JSON/OpenCV YAML."""
    if path.suffix.lower() == '.json':
        with path.open('r', encoding='utf-8') as stream:
            payload = json.load(stream)
        if 'rotation_vector' in payload and 'translation_vector' in payload:
            return (
                np.asarray(payload['rotation_vector'], dtype=np.float64).reshape(3, 1),
                np.asarray(payload['translation_vector'], dtype=np.float64).reshape(
                    3, 1
                ),
            )
        if 'world_to_camera' in payload:
            return matrix_to_extrinsics(np.asarray(payload['world_to_camera']))
        if 'transform_matrix' in payload:
            return matrix_to_extrinsics(np.asarray(payload['transform_matrix']))
        raise SystemExit(f'{path} must contain extrinsic pose data.')

    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not storage.isOpened():
        raise SystemExit(f'Unable to open extrinsics file: {path}')
    try:
        rvec = storage.getNode('rotation_vector').mat()
        tvec = storage.getNode('translation_vector').mat()
        matrix = storage.getNode('world_to_camera').mat()
        if matrix is None:
            matrix = storage.getNode('transform_matrix').mat()
    finally:
        storage.release()

    if rvec is not None and tvec is not None:
        return rvec.astype(np.float64).reshape(3, 1), tvec.astype(np.float64).reshape(
            3, 1
        )
    if matrix is not None:
        return matrix_to_extrinsics(matrix)
    raise SystemExit(f'{path} must contain rotation_vector/translation_vector.')


def matrix_to_extrinsics(matrix):
    """Convert a 4x4 world-to-camera transform into rvec and tvec."""
    transform = np.asarray(matrix, dtype=np.float64)
    if transform.shape != (4, 4):
        raise SystemExit('Extrinsic transform_matrix/world_to_camera must be 4x4.')
    rvec, _jacobian = cv2.Rodrigues(transform[:3, :3])
    tvec = transform[:3, 3].reshape(3, 1)
    return rvec, tvec


class ExtrinsicControls:
    """Wrap OpenCV trackbars for Euler angles and translation."""

    ROTATION_OFFSET = 180

    def __init__(self, translation_range, translation_scale, rvec, tvec):
        self.translation_range = max(1, int(translation_range))
        self.translation_scale = max(1, int(translation_scale))
        self.translation_ticks = int(round(self.translation_range / self.translation_scale))
        self.translation_mid = self.translation_ticks
        self.tz_ticks = self.translation_ticks * 2

        cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CONTROL_WINDOW, 520, 320)
        self._create_trackbars()
        self.set_from_extrinsics(rvec, tvec)

    def _create_trackbars(self):
        for name in ('roll_deg', 'pitch_deg', 'yaw_deg'):
            cv2.createTrackbar(name, CONTROL_WINDOW, self.ROTATION_OFFSET, 360, self._noop)
        cv2.createTrackbar(
            'tx',
            CONTROL_WINDOW,
            self.translation_mid,
            self.translation_ticks * 2,
            self._noop,
        )
        cv2.createTrackbar(
            'ty',
            CONTROL_WINDOW,
            self.translation_mid,
            self.translation_ticks * 2,
            self._noop,
        )
        cv2.createTrackbar('tz', CONTROL_WINDOW, 1, self.tz_ticks, self._noop)

    def set_from_extrinsics(self, rvec, tvec):
        """Move sliders to match rvec/tvec."""
        euler_degrees = rotation_matrix_to_euler_degrees(cv2.Rodrigues(rvec)[0])
        for label, angle in zip(('roll_deg', 'pitch_deg', 'yaw_deg'), euler_degrees):
            cv2.setTrackbarPos(
                label,
                CONTROL_WINDOW,
                self._clamp(angle + self.ROTATION_OFFSET, 0, 360),
            )
        tx, ty, tz = np.asarray(tvec, dtype=np.float64).reshape(3)
        cv2.setTrackbarPos('tx', CONTROL_WINDOW, self._translation_to_pos(tx))
        cv2.setTrackbarPos('ty', CONTROL_WINDOW, self._translation_to_pos(ty))
        cv2.setTrackbarPos('tz', CONTROL_WINDOW, self._tz_to_pos(tz))

    def read(self):
        """Return current rvec, tvec, and Euler angles in degrees."""
        roll = cv2.getTrackbarPos('roll_deg', CONTROL_WINDOW) - self.ROTATION_OFFSET
        pitch = cv2.getTrackbarPos('pitch_deg', CONTROL_WINDOW) - self.ROTATION_OFFSET
        yaw = cv2.getTrackbarPos('yaw_deg', CONTROL_WINDOW) - self.ROTATION_OFFSET
        rotation = euler_degrees_to_rotation_matrix(roll, pitch, yaw)
        rvec, _jacobian = cv2.Rodrigues(rotation)
        tvec = np.array(
            [
                [self._pos_to_translation(cv2.getTrackbarPos('tx', CONTROL_WINDOW))],
                [self._pos_to_translation(cv2.getTrackbarPos('ty', CONTROL_WINDOW))],
                [self._pos_to_tz(cv2.getTrackbarPos('tz', CONTROL_WINDOW))],
            ],
            dtype=np.float64,
        )
        return rvec, tvec, (roll, pitch, yaw)

    def _translation_to_pos(self, value):
        position = int(round(value / self.translation_scale)) + self.translation_mid
        return self._clamp(position, 0, self.translation_ticks * 2)

    def _tz_to_pos(self, value):
        position = int(round(value / self.translation_scale))
        return self._clamp(position, 1, self.tz_ticks)

    def _pos_to_translation(self, position):
        return float((position - self.translation_mid) * self.translation_scale)

    def _pos_to_tz(self, position):
        return float(max(1, position) * self.translation_scale)

    @staticmethod
    def _clamp(value, low, high):
        return int(round(min(max(value, low), high)))

    @staticmethod
    def _noop(_value):
        return None


def euler_degrees_to_rotation_matrix(roll, pitch, yaw):
    """Create a rotation matrix from roll/pitch/yaw in degrees."""
    rx, ry, rz = np.deg2rad([roll, pitch, yaw])
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    rot_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    rot_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rot_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return rot_z @ rot_y @ rot_x


def rotation_matrix_to_euler_degrees(rotation):
    """Convert a rotation matrix to roll/pitch/yaw degrees."""
    sy = np.sqrt(rotation[0, 0] * rotation[0, 0] + rotation[1, 0] * rotation[1, 0])
    singular = sy < 1e-6
    if singular:
        roll = np.arctan2(-rotation[1, 2], rotation[1, 1])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = 0.0
    else:
        roll = np.arctan2(rotation[2, 1], rotation[2, 2])
        pitch = np.arctan2(-rotation[2, 0], sy)
        yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return np.rad2deg([roll, pitch, yaw])


def make_chessboard_points(columns, rows, square_size):
    """Create object points for a planar chessboard target."""
    points = np.zeros((rows * columns, 3), np.float32)
    points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    points *= square_size
    return points


def find_chessboard(frame, pattern_size):
    """Return refined chessboard corners when the target is visible."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        + cv2.CALIB_CB_NORMALIZE_IMAGE
        + cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None
    refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), CRITERIA)
    return True, refined


def make_grid_segments(columns, rows, square_size):
    """Create line segments on the z=0 world plane."""
    segments = []
    width = columns * square_size
    height = rows * square_size
    for column in range(columns + 1):
        x = column * square_size
        segments.append(((x, 0.0, 0.0), (x, height, 0.0)))
    for row in range(rows + 1):
        y = row * square_size
        segments.append(((0.0, y, 0.0), (width, y, 0.0)))
    return np.asarray(segments, dtype=np.float32).reshape(-1, 3)


def draw_projected_grid(frame, rvec, tvec, camera_matrix, distortion, grid_points):
    """Project a planar world grid and coordinate axes onto the preview."""
    output = frame.copy()
    projected, _jacobian = cv2.projectPoints(
        grid_points,
        rvec,
        tvec,
        camera_matrix,
        distortion,
    )
    points = projected.reshape(-1, 2)
    for index in range(0, len(points), 2):
        start = tuple(np.round(points[index]).astype(int))
        end = tuple(np.round(points[index + 1]).astype(int))
        cv2.line(output, start, end, (0, 255, 255), 1, cv2.LINE_AA)

    axis_length = max(1.0, float(np.linalg.norm(grid_points[1] - grid_points[0])))
    axes = np.array(
        [
            [0.0, 0.0, 0.0],
            [axis_length, 0.0, 0.0],
            [0.0, axis_length, 0.0],
            [0.0, 0.0, -axis_length],
        ],
        dtype=np.float32,
    )
    axis_pixels, _jacobian = cv2.projectPoints(
        axes,
        rvec,
        tvec,
        camera_matrix,
        distortion,
    )
    origin, x_axis, y_axis, z_axis = [
        tuple(np.round(point).astype(int)) for point in axis_pixels.reshape(-1, 2)
    ]
    cv2.line(output, origin, x_axis, (0, 0, 255), 3, cv2.LINE_AA)
    cv2.line(output, origin, y_axis, (0, 255, 0), 3, cv2.LINE_AA)
    cv2.line(output, origin, z_axis, (255, 0, 0), 3, cv2.LINE_AA)
    return output


def put_status(frame, rvec, tvec, euler_degrees, found):
    """Add compact extrinsic status text to the preview."""
    tx, ty, tz = tvec.reshape(3)
    rx, ry, rz = rvec.reshape(3)
    roll, pitch, yaw = euler_degrees
    lines = [
        f'roll={roll:+.0f} pitch={pitch:+.0f} yaw={yaw:+.0f} deg  '
        f't=({tx:+.0f}, {ty:+.0f}, {tz:+.0f})',
        f'rvec=({rx:+.3f}, {ry:+.3f}, {rz:+.3f})  '
        f'chessboard: {"found" if found else "not found"}',
        'P/Space: solvePnP snap  G: grid  R: reset  S: save  Q: quit',
    ]
    for index, line in enumerate(lines):
        origin = (12, 28 + index * 28)
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def extrinsics_to_matrix(rvec, tvec):
    """Build a 4x4 world-to-camera transform matrix."""
    rotation, _jacobian = cv2.Rodrigues(rvec)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = tvec.reshape(3)
    return transform


def save_extrinsics(
    path,
    image_size,
    rvec,
    tvec,
    euler_degrees,
    camera_matrix,
    distortion,
):
    """Save extrinsics and intrinsics in OpenCV YAML format."""
    path.parent.mkdir(parents=True, exist_ok=True)
    world_to_camera = extrinsics_to_matrix(rvec, tvec)
    camera_to_world = np.linalg.inv(world_to_camera)
    rotation_matrix = world_to_camera[:3, :3]

    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise SystemExit(f'Unable to write extrinsics file: {path}')
    try:
        storage.write('image_width', int(image_size[0]))
        storage.write('image_height', int(image_size[1]))
        storage.write('camera_matrix', camera_matrix)
        storage.write('distortion_coefficients', distortion)
        storage.write('rotation_vector', rvec)
        storage.write('translation_vector', tvec)
        storage.write('rotation_matrix', rotation_matrix)
        storage.write('euler_degrees_roll_pitch_yaw', np.asarray(euler_degrees))
        storage.write('world_to_camera', world_to_camera)
        storage.write('camera_to_world', camera_to_world)
    finally:
        storage.release()
    print(f'Saved extrinsics to {path}')


def main():
    """Run the live camera extrinsic tuner."""
    args = parse_args()
    pattern_size = (args.columns, args.rows)
    object_points = make_chessboard_points(args.columns, args.rows, args.square_size)
    grid_points = make_grid_segments(
        args.grid_columns,
        args.grid_rows,
        args.square_size,
    )

    cap = open_camera(args)
    grid_enabled = True

    try:
        received, frame = cap.read()
        if not received:
            raise SystemExit(
                f'Camera /dev/video{args.device} opened but returned no frame.'
            )

        image_size = (frame.shape[1], frame.shape[0])
        camera_matrix, distortion = load_intrinsics(args.intrinsics, *image_size)
        if args.load:
            rvec, tvec = load_extrinsics(args.load)
        else:
            rvec, tvec = default_extrinsics()
        default_rvec, default_tvec = default_extrinsics()
        controls = ExtrinsicControls(
            args.translation_range,
            args.translation_scale,
            rvec,
            tvec,
        )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        print('Controls: P/Space solvePnP snap, G grid, R reset, S save, Q/Esc quit')
        if args.intrinsics is None:
            print('No --intrinsics provided; using an approximate camera matrix.')

        while True:
            received, frame = cap.read()
            if not received:
                raise SystemExit('Camera stream stopped returning frames.')

            rvec, tvec, euler_degrees = controls.read()
            found, corners = find_chessboard(frame, pattern_size)

            preview = frame.copy()
            if found:
                cv2.drawChessboardCorners(preview, pattern_size, corners, found)
            if grid_enabled:
                preview = draw_projected_grid(
                    preview,
                    rvec,
                    tvec,
                    camera_matrix,
                    distortion,
                    grid_points,
                )

            put_status(preview, rvec, tvec, euler_degrees, found)
            cv2.imshow(WINDOW_NAME, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('g'):
                grid_enabled = not grid_enabled
            elif key == ord('r'):
                controls.set_from_extrinsics(default_rvec, default_tvec)
            elif key in (ord('p'), ord(' ')):
                if found:
                    ok, pnp_rvec, pnp_tvec = cv2.solvePnP(
                        object_points,
                        corners,
                        camera_matrix,
                        distortion,
                        flags=cv2.SOLVEPNP_ITERATIVE,
                    )
                    if ok:
                        controls.set_from_extrinsics(pnp_rvec, pnp_tvec)
                        print('Updated extrinsics from chessboard solvePnP.')
                else:
                    print('Chessboard not found; cannot solvePnP.')
            elif key == ord('s'):
                save_extrinsics(
                    args.output,
                    image_size,
                    rvec,
                    tvec,
                    euler_degrees,
                    camera_matrix,
                    distortion,
                )
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
