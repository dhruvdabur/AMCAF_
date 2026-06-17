#!/usr/bin/env python3
"""Test a V4L2 camera, preview it, and publish frames on ROS2."""

import argparse
import re
import subprocess
import time
from pathlib import Path

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image


WINDOW_NAME = 'Camera Test'
CONTROL_WINDOW = 'Camera Controls'
CONTROL_NAMES = (
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
OPENCV_CONTROLS = {
    'brightness': cv2.CAP_PROP_BRIGHTNESS,
    'contrast': cv2.CAP_PROP_CONTRAST,
    'saturation': cv2.CAP_PROP_SATURATION,
    'hue': cv2.CAP_PROP_HUE,
    'gain': cv2.CAP_PROP_GAIN,
    'exposure': cv2.CAP_PROP_EXPOSURE,
}


def parse_args(args=None):
    """Read camera, ROS topic, and output settings."""
    parser = argparse.ArgumentParser(
        description='Camera testing utility with ROS2 /image_raw publishing.'
    )
    parser.add_argument(
        '--device',
        type=int,
        default=2,
        help='Video device index (default: 2).',
    )
    parser.add_argument(
        '--width',
        type=int,
        default=1920,
        help='Requested capture width (default: 1920).',
    )
    parser.add_argument(
        '--height',
        type=int,
        default=1080,
        help='Requested capture height (default: 1080).',
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
        help='Requested pixel format (default: MJPG).',
    )
    parser.add_argument(
        '--topic',
        default='/image_raw',
        help='ROS2 image topic to publish (default: /image_raw).',
    )
    parser.add_argument(
        '--camera-info',
        type=Path,
        default=None,
        help='ROS camera_info YAML to publish with each image.',
    )
    parser.add_argument(
        '--camera-info-topic',
        default='/camera_info',
        help='ROS2 camera info topic to publish when --camera-info is set.',
    )
    parser.add_argument(
        '--frame-id',
        default='camera',
        help='ROS frame_id for published images (default: camera).',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('camera_testing.yaml'),
        help='Path for saved camera properties (default: camera_testing.yaml).',
    )
    parser.add_argument(
        '--skip-v4l2-ctl',
        action='store_true',
        help='Skip v4l2-ctl format listing and control commands.',
    )
    parser.add_argument(
        '--controls',
        action='store_true',
        help='Open a live brightness/exposure control slider window.',
    )
    parser.add_argument(
        '--no-preview',
        action='store_true',
        help='Publish frames without opening the OpenCV preview window.',
    )
    return parser.parse_known_args(args)


class CameraTester(Node):
    """Open a camera, show a preview, and publish frames as ROS images."""

    def __init__(self, config):
        """Configure the selected camera and create the ROS publisher."""
        super().__init__('camera_tester_node')
        self.config = config
        self.device_path = f'/dev/video{config.device}'
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(
            Image,
            config.topic,
            make_sensor_qos(),
        )
        self.camera_info_publisher = None
        self.camera_info = None
        if config.camera_info is not None:
            self.camera_info = load_camera_info(config.camera_info, config.frame_id)
            self.camera_info_publisher = self.create_publisher(
                CameraInfo,
                config.camera_info_topic,
                make_sensor_qos(),
            )
        self.cap = None
        self.controls = {}
        self.trackbar_to_control = {}

        print('=' * 60)
        print('Camera Testing Utility')
        print('=' * 60)

        if not config.skip_v4l2_ctl:
            self.configure_with_v4l2_ctl()

        print(f'\nOpening camera {self.device_path}...')
        self.cap = cv2.VideoCapture(self.device_path, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'Failed to open camera {self.device_path}')

        self.configure_with_opencv()
        self.print_camera_info()
        if config.controls:
            self.create_control_panel()

    def configure_with_v4l2_ctl(self):
        """Print V4L2 formats and request the configured camera mode."""
        try:
            result = subprocess.run(
                ['v4l2-ctl', '-d', self.device_path, '--list-formats-ext'],
                capture_output=True,
                text=True,
                check=True,
            )
            print('\nAvailable camera formats:')
            print(result.stdout)

            print(
                f'\nAttempting to set camera to '
                f'{self.config.width}x{self.config.height} {self.config.format}...'
            )
            subprocess.run(
                [
                    'v4l2-ctl',
                    '-d',
                    self.device_path,
                    (
                        '--set-fmt-video='
                        f'width={self.config.width},'
                        f'height={self.config.height},'
                        f'pixelformat={self.config.format}'
                    ),
                ],
                check=False,
            )
            subprocess.run(
                ['v4l2-ctl', '-d', self.device_path, '-c', 'auto_exposure=3'],
                check=False,
            )
        except FileNotFoundError:
            print('Warning: v4l2-ctl is not installed; using OpenCV settings only.')
        except subprocess.CalledProcessError as error:
            print(f'Warning: v4l2-ctl command failed: {error}')

    def configure_with_opencv(self):
        """Request camera format and image size through OpenCV."""
        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*self.config.format),
        )
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)

    def print_camera_info(self):
        """Print current OpenCV camera properties."""
        print('\n' + '=' * 60)
        print('Camera Properties:')
        print('=' * 60)

        properties = {
            'Frame Width': cv2.CAP_PROP_FRAME_WIDTH,
            'Frame Height': cv2.CAP_PROP_FRAME_HEIGHT,
            'FPS': cv2.CAP_PROP_FPS,
            'Format': cv2.CAP_PROP_FORMAT,
            'FOURCC': cv2.CAP_PROP_FOURCC,
            'Brightness': cv2.CAP_PROP_BRIGHTNESS,
            'Contrast': cv2.CAP_PROP_CONTRAST,
            'Saturation': cv2.CAP_PROP_SATURATION,
            'Hue': cv2.CAP_PROP_HUE,
            'Gain': cv2.CAP_PROP_GAIN,
            'Exposure': cv2.CAP_PROP_EXPOSURE,
            'Auto Exposure': cv2.CAP_PROP_AUTO_EXPOSURE,
            'Backend': cv2.CAP_PROP_BACKEND,
            'Buffer Size': cv2.CAP_PROP_BUFFERSIZE,
        }

        for name, prop in properties.items():
            value = self.cap.get(prop)
            if name == 'FOURCC' and value > 0:
                fourcc_str = fourcc_to_string(value)
                print(f'{name:20s}: {int(value)} ({fourcc_str})')
            else:
                print(f'{name:20s}: {value}')

        print('=' * 60)

    def get_camera_properties(self):
        """Get current camera properties as a YAML-friendly dictionary."""
        properties = {
            'device': self.device_path,
            'ros_topic': self.config.topic,
            'frame_id': self.config.frame_id,
            'Frame Width': int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
            'Frame Height': int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            'FPS': float(self.cap.get(cv2.CAP_PROP_FPS)),
            'Format': int(self.cap.get(cv2.CAP_PROP_FORMAT)),
            'FOURCC': int(self.cap.get(cv2.CAP_PROP_FOURCC)),
            'Brightness': float(self.cap.get(cv2.CAP_PROP_BRIGHTNESS)),
            'Contrast': float(self.cap.get(cv2.CAP_PROP_CONTRAST)),
            'Saturation': float(self.cap.get(cv2.CAP_PROP_SATURATION)),
            'Hue': float(self.cap.get(cv2.CAP_PROP_HUE)),
            'Gain': float(self.cap.get(cv2.CAP_PROP_GAIN)),
            'Exposure': float(self.cap.get(cv2.CAP_PROP_EXPOSURE)),
            'Auto Exposure': float(self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE)),
            'Backend': int(self.cap.get(cv2.CAP_PROP_BACKEND)),
            'Buffer Size': int(self.cap.get(cv2.CAP_PROP_BUFFERSIZE)),
        }

        if properties['FOURCC'] > 0:
            properties['FOURCC_String'] = fourcc_to_string(properties['FOURCC'])

        properties['controls'] = self.get_control_values()
        return properties

    def create_control_panel(self):
        """Create sliders for camera controls that this device exposes."""
        cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(CONTROL_WINDOW, 520, 420)

        if not self.config.skip_v4l2_ctl:
            self.controls = self.read_v4l2_controls()

        if self.controls:
            for name, control in self.controls.items():
                label = control['label']
                cv2.createTrackbar(
                    label,
                    CONTROL_WINDOW,
                    self.value_to_position(control, control['value']),
                    control['max_position'],
                    self.noop,
                )
                self.trackbar_to_control[label] = name
            print(f'Live camera controls available in "{CONTROL_WINDOW}".')
            return

        print('Using basic OpenCV camera controls; ranges may vary by camera.')
        for name, prop in OPENCV_CONTROLS.items():
            current = int(round(self.cap.get(prop)))
            value = min(max(current, 0), 255)
            label = name
            self.controls[name] = {
                'label': label,
                'backend': 'opencv',
                'property': prop,
                'min': 0,
                'step': 1,
                'max_position': 255,
                'last_value': value,
            }
            cv2.createTrackbar(label, CONTROL_WINDOW, value, 255, self.noop)
            self.trackbar_to_control[label] = name

    def read_v4l2_controls(self):
        """Read supported V4L2 controls and their ranges."""
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
            if name not in CONTROL_NAMES:
                continue
            low = int(match.group('min'))
            high = int(match.group('max'))
            step = max(1, int(match.group('step')))
            value = int(match.group('value'))
            controls[name] = {
                'label': name[:30],
                'backend': 'v4l2',
                'min': low,
                'max': high,
                'step': step,
                'value': value,
                'last_value': value,
                'max_position': int((high - low) / step),
            }
        return controls

    def update_camera_controls(self):
        """Apply changed slider values to the active camera."""
        for name, control in self.controls.items():
            label = control['label']
            position = cv2.getTrackbarPos(label, CONTROL_WINDOW)
            value = self.position_to_value(control, position)
            if value == control.get('last_value'):
                continue

            if control['backend'] == 'v4l2':
                self.set_v4l2_control(name, value)
            else:
                self.cap.set(control['property'], value)
            control['last_value'] = value

    def set_v4l2_control(self, name, value):
        """Set one V4L2 control; keep streaming even if the driver rejects it."""
        result = subprocess.run(
            [
                'v4l2-ctl',
                '-d',
                self.device_path,
                '-c',
                f'{name}={value}',
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            print(f'Warning: could not set {name}={value}: {result.stderr.strip()}')

    def get_control_values(self):
        """Return current live-control slider values."""
        values = {}
        for name, control in self.controls.items():
            if control['backend'] == 'v4l2':
                values[name] = control.get('last_value')
            else:
                values[name] = float(self.cap.get(control['property']))
        return values

    @staticmethod
    def value_to_position(control, value):
        """Convert a camera control value into a trackbar position."""
        return int(round((value - control['min']) / control['step']))

    @staticmethod
    def position_to_value(control, position):
        """Convert a trackbar position into a camera control value."""
        return int(control['min'] + position * control['step'])

    @staticmethod
    def noop(_value):
        """OpenCV trackbar callback placeholder."""
        return None

    def save_properties_to_yaml(self):
        """Save camera runtime properties to YAML."""
        self.config.output.parent.mkdir(parents=True, exist_ok=True)
        with self.config.output.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                self.get_camera_properties(),
                stream,
                default_flow_style=False,
                sort_keys=False,
            )

        print(f'\nCamera properties saved to {self.config.output}')

    def publish_frame(self, frame):
        """Publish one BGR OpenCV frame as a ROS Image message."""
        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = self.config.frame_id
        self.image_publisher.publish(ros_image)
        if self.camera_info is not None:
            self.camera_info.header.stamp = ros_image.header.stamp
            self.camera_info.header.frame_id = ros_image.header.frame_id
            self.camera_info_publisher.publish(self.camera_info)

    def run(self):
        """Show the video stream until quit and publish each frame."""
        print('\nStarting video stream...')
        print("Press 'q' to quit, 'i' to print camera info again")
        print(f'Publishing to ROS2 topic: {self.config.topic}')
        if self.camera_info is not None:
            print(f'Publishing camera info to: {self.config.camera_info_topic}')

        frame_count = 0
        start_time = time.time()
        if not self.config.no_preview:
            cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

        while rclpy.ok():
            if self.config.controls:
                self.update_camera_controls()
            received, frame = self.cap.read()
            if not received:
                print(
                    'Failed to capture frame. If this happens after one frame, '
                    'check the V4L2 auto_exposure control.'
                )
                break

            frame_count += 1
            elapsed_time = time.time() - start_time
            actual_fps = frame_count / elapsed_time if elapsed_time > 0 else 0.0

            if not self.config.no_preview:
                display_frame = frame.copy()
                add_overlay(display_frame, frame_count, actual_fps)
                cv2.imshow(WINDOW_NAME, display_frame)

            try:
                self.publish_frame(frame)
            except Exception as error:
                self.get_logger().error(f'Failed to publish image: {error}')

            rclpy.spin_once(self, timeout_sec=0.0)

            if not self.config.no_preview:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    self.save_properties_to_yaml()
                    break
                if key == ord('i'):
                    self.print_camera_info()

            if frame_count % 30 == 0:
                print(f'Frames: {frame_count} | Actual FPS: {actual_fps:.2f}')

        print(f'\nTest completed. Total frames: {frame_count}')
        if frame_count > 0 and elapsed_time > 0:
            print(f'Average FPS: {frame_count / elapsed_time:.2f}')

    def close(self):
        """Release camera and GUI resources."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()


def fourcc_to_string(value):
    """Convert an OpenCV FOURCC numeric value into four characters."""
    return ''.join(chr((int(value) >> (8 * index)) & 0xFF) for index in range(4))


def make_sensor_qos():
    """Create low-latency QoS that keeps only the newest image."""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
    )


def load_camera_info(path, frame_id):
    """Load a ROS camera_info YAML file into a CameraInfo message."""
    payload = yaml.safe_load(path.read_text(encoding='utf-8'))
    camera_info = CameraInfo()
    camera_info.header.frame_id = frame_id
    camera_info.width = int(payload['image_width'])
    camera_info.height = int(payload['image_height'])
    camera_info.distortion_model = payload.get('distortion_model', 'plumb_bob')
    camera_info.d = list(payload['distortion_coefficients']['data'])
    camera_info.k = list(payload['camera_matrix']['data'])
    camera_info.r = list(payload['rectification_matrix']['data'])
    camera_info.p = list(payload['projection_matrix']['data'])
    return camera_info


def add_overlay(frame, frame_count, actual_fps):
    """Draw current stream stats on a preview frame."""
    gray_mean = cv2.mean(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))[0]
    height, width = frame.shape[:2]
    info_text = [
        f'Resolution: {width}x{height}',
        f'Actual FPS: {actual_fps:.2f}',
        f'Frame: {frame_count}',
        f'Brightness: {gray_mean:.1f}',
    ]

    for index, text in enumerate(info_text):
        y_offset = 30 + index * 30
        cv2.putText(
            frame,
            text,
            (10, y_offset),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )


def main(args=None):
    """Run the ROS2 camera tester."""
    config, ros_args = parse_args(args)
    rclpy.init(args=ros_args)
    tester = None

    try:
        tester = CameraTester(config)
        tester.run()
    except KeyboardInterrupt:
        print('\nTest interrupted by user')
        if tester is not None:
            tester.save_properties_to_yaml()
    except Exception as error:
        print(f'Error: {error}')
    finally:
        if tester is not None:
            tester.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
