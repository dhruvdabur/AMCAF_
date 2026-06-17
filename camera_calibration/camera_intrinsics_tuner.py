#!/usr/bin/env python3
"""Interactively tune camera intrinsic and distortion parameters."""

import argparse
import json
import re
import subprocess
from pathlib import Path

import cv2
import numpy as np


WINDOW_NAME = 'Camera Intrinsics Tuner'
CONTROL_WINDOW = 'Intrinsic Controls'
IMAGE_CONTROL_WINDOW = 'Image Controls'
CAMERA_CONTROL_NAMES = (
    'brightness',
    'contrast',
    'saturation',
    'hue',
    'white_balance_temperature',
    'sharpness',
    'gain',
    'auto_exposure',
    'exposure_absolute',
    'exposure_time_absolute',
)
OPENCV_CAMERA_CONTROLS = {
    'brightness': cv2.CAP_PROP_BRIGHTNESS,
    'contrast': cv2.CAP_PROP_CONTRAST,
    'saturation': cv2.CAP_PROP_SATURATION,
    'hue': cv2.CAP_PROP_HUE,
    'gain': cv2.CAP_PROP_GAIN,
    'exposure': cv2.CAP_PROP_EXPOSURE,
    'auto_exposure': cv2.CAP_PROP_AUTO_EXPOSURE,
}


def parse_args():
    """Read camera selection and tuner settings."""
    parser = argparse.ArgumentParser(
        description=(
            'Open a live camera preview with sliders for fx, fy, cx, cy, '
            'and OpenCV distortion coefficients.'
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
        '--load',
        type=Path,
        help='Load initial parameters from a JSON or OpenCV YAML file.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('camera_intrinsics.yaml'),
        help='Path to save tuned parameters with S (default: camera_intrinsics.yaml).',
    )
    parser.add_argument(
        '--no-camera-controls',
        action='store_true',
        help='Skip V4L2/OpenCV hardware camera sliders; preview sliders stay available.',
    )
    return parser.parse_args()


def default_parameters(width, height):
    """Return a reasonable starting camera matrix and zero distortion."""
    focal = float(max(width, height))
    camera_matrix = np.array(
        [[focal, 0.0, width / 2.0], [0.0, focal, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.zeros((5, 1), dtype=np.float64)
    return camera_matrix, distortion


def load_parameters(path, width, height):
    """Load camera parameters from JSON or OpenCV YAML."""
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
        raise SystemExit(f'Unable to open calibration file: {path}')
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


class TunerControls:
    """Wrap OpenCV trackbars and convert slider positions to camera values."""

    DISTORTION_SCALE = 1000.0
    TANGENTIAL_SCALE = 10000.0

    def __init__(self, width, height, camera_matrix, distortion):
        self.width = width
        self.height = height
        self.max_focal = max(width, height) * 4
        self.max_cx = width * 2
        self.max_cy = height * 2
        self.distortion_mid = int(2 * self.DISTORTION_SCALE)
        self.tangential_mid = int(1 * self.TANGENTIAL_SCALE)

        cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CONTROL_WINDOW, 560, 360)
        self._create_trackbars()
        self.set_from_parameters(camera_matrix, distortion, alpha=0)

    def _create_trackbars(self):
        cv2.createTrackbar('fx', CONTROL_WINDOW, 1, self.max_focal, self._noop)
        cv2.createTrackbar('fy', CONTROL_WINDOW, 1, self.max_focal, self._noop)
        cv2.createTrackbar('cx', CONTROL_WINDOW, 0, self.max_cx, self._noop)
        cv2.createTrackbar('cy', CONTROL_WINDOW, 0, self.max_cy, self._noop)
        cv2.createTrackbar(
            'k1',
            CONTROL_WINDOW,
            self.distortion_mid,
            self.distortion_mid * 2,
            self._noop,
        )
        cv2.createTrackbar(
            'k2',
            CONTROL_WINDOW,
            self.distortion_mid,
            self.distortion_mid * 2,
            self._noop,
        )
        cv2.createTrackbar(
            'p1',
            CONTROL_WINDOW,
            self.tangential_mid,
            self.tangential_mid * 2,
            self._noop,
        )
        cv2.createTrackbar(
            'p2',
            CONTROL_WINDOW,
            self.tangential_mid,
            self.tangential_mid * 2,
            self._noop,
        )
        cv2.createTrackbar(
            'k3',
            CONTROL_WINDOW,
            self.distortion_mid,
            self.distortion_mid * 2,
            self._noop,
        )
        cv2.createTrackbar('alpha', CONTROL_WINDOW, 0, 100, self._noop)

    def set_from_parameters(self, camera_matrix, distortion, alpha=None):
        """Move sliders to match a camera matrix and distortion vector."""
        cv2.setTrackbarPos(
            'fx',
            CONTROL_WINDOW,
            self._clamp(camera_matrix[0, 0], 1, self.max_focal),
        )
        cv2.setTrackbarPos(
            'fy',
            CONTROL_WINDOW,
            self._clamp(camera_matrix[1, 1], 1, self.max_focal),
        )
        cv2.setTrackbarPos(
            'cx',
            CONTROL_WINDOW,
            self._clamp(camera_matrix[0, 2], 0, self.max_cx),
        )
        cv2.setTrackbarPos(
            'cy',
            CONTROL_WINDOW,
            self._clamp(camera_matrix[1, 2], 0, self.max_cy),
        )
        coefficients = normalize_distortion(distortion).reshape(-1)
        cv2.setTrackbarPos(
            'k1',
            CONTROL_WINDOW,
            self._clamp(
                coefficients[0] * self.DISTORTION_SCALE + self.distortion_mid,
                0,
                self.distortion_mid * 2,
            ),
        )
        cv2.setTrackbarPos(
            'k2',
            CONTROL_WINDOW,
            self._clamp(
                coefficients[1] * self.DISTORTION_SCALE + self.distortion_mid,
                0,
                self.distortion_mid * 2,
            ),
        )
        cv2.setTrackbarPos(
            'p1',
            CONTROL_WINDOW,
            self._clamp(
                coefficients[2] * self.TANGENTIAL_SCALE + self.tangential_mid,
                0,
                self.tangential_mid * 2,
            ),
        )
        cv2.setTrackbarPos(
            'p2',
            CONTROL_WINDOW,
            self._clamp(
                coefficients[3] * self.TANGENTIAL_SCALE + self.tangential_mid,
                0,
                self.tangential_mid * 2,
            ),
        )
        cv2.setTrackbarPos(
            'k3',
            CONTROL_WINDOW,
            self._clamp(
                coefficients[4] * self.DISTORTION_SCALE + self.distortion_mid,
                0,
                self.distortion_mid * 2,
            ),
        )
        if alpha is not None:
            cv2.setTrackbarPos('alpha', CONTROL_WINDOW, self._clamp(alpha, 0, 100))

    def read(self):
        """Return current camera matrix, distortion coefficients, and alpha."""
        fx = max(1.0, cv2.getTrackbarPos('fx', CONTROL_WINDOW))
        fy = max(1.0, cv2.getTrackbarPos('fy', CONTROL_WINDOW))
        cx = float(cv2.getTrackbarPos('cx', CONTROL_WINDOW))
        cy = float(cv2.getTrackbarPos('cy', CONTROL_WINDOW))
        k1 = self._read_distortion('k1')
        k2 = self._read_distortion('k2')
        p1 = self._read_tangential('p1')
        p2 = self._read_tangential('p2')
        k3 = self._read_distortion('k3')
        alpha = cv2.getTrackbarPos('alpha', CONTROL_WINDOW) / 100.0

        camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64
        )
        distortion = np.array([[k1], [k2], [p1], [p2], [k3]], dtype=np.float64)
        return camera_matrix, distortion, alpha

    def _read_distortion(self, name):
        value = cv2.getTrackbarPos(name, CONTROL_WINDOW)
        return (value - self.distortion_mid) / self.DISTORTION_SCALE

    def _read_tangential(self, name):
        value = cv2.getTrackbarPos(name, CONTROL_WINDOW)
        return (value - self.tangential_mid) / self.TANGENTIAL_SCALE

    @staticmethod
    def _clamp(value, low, high):
        return int(round(min(max(value, low), high)))

    @staticmethod
    def _noop(_value):
        return None


class ImageControls:
    """Track hardware camera controls and preview-only image adjustments."""

    PREVIEW_DEFAULTS = {
        'view_brightness': 100,
        'view_contrast_x100': 100,
        'view_gamma_x100': 100,
        'view_saturation_x100': 100,
        'view_hue_shift': 180,
        'view_sharpness': 0,
        'view_blur': 0,
        'view_clahe': 0,
    }
    PREVIEW_MAX = {
        'view_brightness': 200,
        'view_contrast_x100': 300,
        'view_gamma_x100': 300,
        'view_saturation_x100': 300,
        'view_hue_shift': 360,
        'view_sharpness': 100,
        'view_blur': 30,
        'view_clahe': 1,
    }

    def __init__(self, cap, device, include_camera_controls=True):
        self.cap = cap
        self.device_path = f'/dev/video{device}'
        self.camera_controls = {}
        self.trackbar_to_control = {}

        cv2.namedWindow(IMAGE_CONTROL_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(IMAGE_CONTROL_WINDOW, 620, 520)

        if include_camera_controls:
            self._create_camera_control_trackbars()
        self._create_preview_trackbars()

    def _create_camera_control_trackbars(self):
        self.camera_controls = self._read_v4l2_controls()
        if self.camera_controls:
            for name, control in self.camera_controls.items():
                label = f'cam_{control["label"]}'[:30]
                control['label'] = label
                cv2.createTrackbar(
                    label,
                    IMAGE_CONTROL_WINDOW,
                    self._value_to_position(control, control['value']),
                    control['max_position'],
                    self._noop,
                )
                self.trackbar_to_control[label] = name
            return

        print('Using basic OpenCV camera controls; ranges may vary by camera.')
        for name, prop in OPENCV_CAMERA_CONTROLS.items():
            current = int(round(self.cap.get(prop)))
            if current < 0:
                current = 0
            value = min(max(current, 0), 255)
            label = f'cam_{name}'[:30]
            self.camera_controls[name] = {
                'label': label,
                'backend': 'opencv',
                'property': prop,
                'min': 0,
                'step': 1,
                'max_position': 255,
                'last_value': value,
            }
            cv2.createTrackbar(label, IMAGE_CONTROL_WINDOW, value, 255, self._noop)
            self.trackbar_to_control[label] = name

    def _create_preview_trackbars(self):
        for label, value in self.PREVIEW_DEFAULTS.items():
            cv2.createTrackbar(
                label,
                IMAGE_CONTROL_WINDOW,
                value,
                self.PREVIEW_MAX[label],
                self._noop,
            )

    def update_camera_controls(self):
        """Apply changed hardware slider values to the active camera."""
        for name, control in self.camera_controls.items():
            label = control['label']
            position = cv2.getTrackbarPos(label, IMAGE_CONTROL_WINDOW)
            value = self._position_to_value(control, position)
            if value == control.get('last_value'):
                continue

            if control['backend'] == 'v4l2':
                self._set_v4l2_control(name, value)
            else:
                self.cap.set(control['property'], value)
            control['last_value'] = value

    def apply_preview_adjustments(self, frame):
        """Apply preview-only image operations before calibration visualization."""
        settings = self.read_preview_settings()
        adjusted = frame

        blur = settings['blur']
        if blur > 0:
            kernel = blur * 2 + 1
            adjusted = cv2.GaussianBlur(adjusted, (kernel, kernel), 0)

        contrast = settings['contrast']
        brightness = settings['brightness']
        if contrast != 1.0 or brightness != 0:
            adjusted = np.clip(
                adjusted.astype(np.float32) * contrast + brightness,
                0,
                255,
            ).astype(np.uint8)

        gamma = settings['gamma']
        if abs(gamma - 1.0) > 0.001:
            adjusted = self._apply_gamma(adjusted, gamma)

        if (
            settings['saturation'] != 1.0
            or settings['hue_shift'] != 0
            or settings['clahe']
        ):
            hsv = cv2.cvtColor(adjusted, cv2.COLOR_BGR2HSV)
            if settings['hue_shift'] != 0:
                hue = hsv[:, :, 0].astype(np.int16)
                hsv[:, :, 0] = np.mod(hue + settings['hue_shift'], 180).astype(
                    np.uint8
                )
            if settings['saturation'] != 1.0:
                saturation = hsv[:, :, 1].astype(np.float32) * settings['saturation']
                hsv[:, :, 1] = np.clip(saturation, 0, 255).astype(np.uint8)
            if settings['clahe']:
                clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
                hsv[:, :, 2] = clahe.apply(hsv[:, :, 2])
            adjusted = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

        sharpness = settings['sharpness']
        if sharpness > 0:
            blurred = cv2.GaussianBlur(adjusted, (0, 0), 1.2)
            adjusted = cv2.addWeighted(adjusted, 1.0 + sharpness, blurred, -sharpness, 0)

        return adjusted

    def read_preview_settings(self):
        """Return current preview adjustment values in human-scale units."""
        return {
            'brightness': cv2.getTrackbarPos(
                'view_brightness', IMAGE_CONTROL_WINDOW
            )
            - 100,
            'contrast': cv2.getTrackbarPos(
                'view_contrast_x100', IMAGE_CONTROL_WINDOW
            )
            / 100.0,
            'gamma': max(
                1,
                cv2.getTrackbarPos('view_gamma_x100', IMAGE_CONTROL_WINDOW),
            )
            / 100.0,
            'saturation': cv2.getTrackbarPos(
                'view_saturation_x100', IMAGE_CONTROL_WINDOW
            )
            / 100.0,
            'hue_shift': cv2.getTrackbarPos(
                'view_hue_shift', IMAGE_CONTROL_WINDOW
            )
            - 180,
            'sharpness': cv2.getTrackbarPos(
                'view_sharpness', IMAGE_CONTROL_WINDOW
            )
            / 25.0,
            'blur': cv2.getTrackbarPos('view_blur', IMAGE_CONTROL_WINDOW),
            'clahe': bool(cv2.getTrackbarPos('view_clahe', IMAGE_CONTROL_WINDOW)),
        }

    def get_camera_control_values(self):
        """Return last applied hardware camera control values."""
        values = {}
        for name, control in self.camera_controls.items():
            if control['backend'] == 'v4l2':
                values[name] = control.get('last_value')
            else:
                values[name] = float(self.cap.get(control['property']))
        return values

    def reset_preview_adjustments(self):
        """Reset preview-only sliders to neutral values."""
        for label, value in self.PREVIEW_DEFAULTS.items():
            cv2.setTrackbarPos(label, IMAGE_CONTROL_WINDOW, value)

    def _read_v4l2_controls(self):
        try:
            result = subprocess.run(
                ['v4l2-ctl', '-d', self.device_path, '--list-ctrls'],
                capture_output=True,
                text=True,
                check=True,
            )
        except (FileNotFoundError, subprocess.CalledProcessError):
            return {}

        controls = {}
        pattern = re.compile(
            r'^\s*(?P<name>\w+)\s+.*:\s+'
            r'min=(?P<min>-?\d+)\s+'
            r'max=(?P<max>-?\d+)\s+'
            r'step=(?P<step>\d+)\s+'
            r'default=(?P<default>-?\d+)\s+'
            r'value=(?P<value>-?\d+)'
        )
        for line in result.stdout.splitlines():
            match = pattern.match(line)
            if not match:
                continue
            name = match.group('name')
            if name not in CAMERA_CONTROL_NAMES:
                continue
            low = int(match.group('min'))
            high = int(match.group('max'))
            step = max(1, int(match.group('step')))
            value = int(match.group('value'))
            controls[name] = {
                'label': name,
                'backend': 'v4l2',
                'min': low,
                'max': high,
                'step': step,
                'value': value,
                'last_value': value,
                'max_position': int((high - low) / step),
            }
        return controls

    def _set_v4l2_control(self, name, value):
        result = subprocess.run(
            ['v4l2-ctl', '-d', self.device_path, '-c', f'{name}={value}'],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            print(f'Warning: could not set {name}={value}: {result.stderr.strip()}')

    @staticmethod
    def _apply_gamma(frame, gamma):
        inverse = 1.0 / gamma
        table = np.array(
            [((value / 255.0) ** inverse) * 255 for value in range(256)],
            dtype=np.uint8,
        )
        return cv2.LUT(frame, table)

    @staticmethod
    def _value_to_position(control, value):
        return int(round((value - control['min']) / control['step']))

    @staticmethod
    def _position_to_value(control, position):
        return int(control['min'] + position * control['step'])

    @staticmethod
    def _noop(_value):
        return None


def draw_grid(frame, spacing=80):
    """Draw a reference grid over the preview frame."""
    output = frame.copy()
    height, width = output.shape[:2]
    for x in range(0, width, spacing):
        cv2.line(output, (x, 0), (x, height), (0, 255, 255), 1, cv2.LINE_AA)
    for y in range(0, height, spacing):
        cv2.line(output, (0, y), (width, y), (0, 255, 255), 1, cv2.LINE_AA)
    return cv2.addWeighted(output, 0.35, frame, 0.65, 0.0)


def put_status(frame, camera_matrix, distortion, undistort_enabled):
    """Add compact parameter text to the preview."""
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]
    k1, k2, p1, p2, k3 = distortion.reshape(-1)
    lines = [
        f'fx={fx:.0f} fy={fy:.0f} cx={cx:.0f} cy={cy:.0f}',
        f'k1={k1:+.3f} k2={k2:+.3f} p1={p1:+.4f} p2={p2:+.4f} k3={k3:+.3f}',
        f'U: undistort {"on" if undistort_enabled else "off"}  '
        'G: grid  R: reset intrinsics  A: reset image  S: save  Q: quit',
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


def save_parameters(
    path,
    camera_matrix,
    distortion,
    image_size,
    preview_settings=None,
    camera_controls=None,
):
    """Save current camera parameters in OpenCV YAML format."""
    path.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise SystemExit(f'Unable to write calibration file: {path}')
    try:
        storage.write('image_width', int(image_size[0]))
        storage.write('image_height', int(image_size[1]))
        storage.write('camera_matrix', camera_matrix)
        storage.write('distortion_coefficients', distortion)
        if preview_settings:
            storage.write('preview_brightness', float(preview_settings['brightness']))
            storage.write('preview_contrast', float(preview_settings['contrast']))
            storage.write('preview_gamma', float(preview_settings['gamma']))
            storage.write('preview_saturation', float(preview_settings['saturation']))
            storage.write('preview_hue_shift', int(preview_settings['hue_shift']))
            storage.write('preview_sharpness', float(preview_settings['sharpness']))
            storage.write('preview_blur', int(preview_settings['blur']))
            storage.write('preview_clahe', int(preview_settings['clahe']))
        if camera_controls:
            for name, value in camera_controls.items():
                storage.write(f'camera_control_{name}', float(value))
    finally:
        storage.release()
    print(f'Saved intrinsics to {path}')


def main():
    """Run the live camera intrinsic tuner."""
    args = parse_args()
    cap = open_camera(args)
    undistort_enabled = True
    grid_enabled = True

    try:
        received, frame = cap.read()
        if not received:
            raise SystemExit(
                f'Camera /dev/video{args.device} opened but returned no frame.'
            )

        height, width = frame.shape[:2]
        camera_matrix, distortion = (
            load_parameters(args.load, width, height)
            if args.load
            else default_parameters(width, height)
        )
        default_matrix, default_distortion = default_parameters(width, height)
        controls = TunerControls(width, height, camera_matrix, distortion)
        image_controls = ImageControls(
            cap,
            args.device,
            include_camera_controls=not args.no_camera_controls,
        )

        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        print(
            'Controls: U undistort, G grid, R reset intrinsics, '
            'A reset image, S save, Q/Esc quit'
        )

        while True:
            image_controls.update_camera_controls()
            camera_matrix, distortion, alpha = controls.read()
            adjusted_frame = image_controls.apply_preview_adjustments(frame)
            preview = adjusted_frame
            if undistort_enabled:
                new_matrix, _roi = cv2.getOptimalNewCameraMatrix(
                    camera_matrix, distortion, (width, height), alpha, (width, height)
                )
                preview = cv2.undistort(
                    adjusted_frame,
                    camera_matrix,
                    distortion,
                    None,
                    new_matrix,
                )
            if grid_enabled:
                preview = draw_grid(preview)

            put_status(preview, camera_matrix, distortion, undistort_enabled)
            cv2.imshow(WINDOW_NAME, preview)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('u'):
                undistort_enabled = not undistort_enabled
            elif key == ord('g'):
                grid_enabled = not grid_enabled
            elif key == ord('r'):
                controls.set_from_parameters(
                    default_matrix,
                    default_distortion,
                    alpha=0,
                )
            elif key == ord('s'):
                save_parameters(
                    args.output,
                    camera_matrix,
                    distortion,
                    (width, height),
                    image_controls.read_preview_settings(),
                    image_controls.get_camera_control_values(),
                )
            elif key == ord('a'):
                image_controls.reset_preview_adjustments()

            received, frame = cap.read()
            if not received:
                raise SystemExit('Camera stream stopped returning frames.')
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
