#!/usr/bin/env python3
"""Calibrate camera intrinsics from a live chessboard/checkerboard target."""

import argparse
from pathlib import Path

import cv2
import numpy as np


WINDOW_NAME = 'Chessboard Camera Calibrator'
CRITERIA = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30,
    0.001,
)


def parse_args():
    """Read camera, board, and output settings."""
    parser = argparse.ArgumentParser(
        description=(
            'Automatically estimate camera intrinsics from a printed '
            'chessboard/checkerboard calibration target.'
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
        '--columns',
        type=int,
        default=9,
        help='Number of inner chessboard corners across columns (default: 9).',
    )
    parser.add_argument(
        '--rows',
        type=int,
        default=6,
        help='Number of inner chessboard corners across rows (default: 6).',
    )
    parser.add_argument(
        '--square-size',
        type=float,
        default=1.0,
        help=(
            'Physical size of one square. Units are arbitrary but consistent, '
            'for example 25.0 for 25 mm squares. Default: 1.0.'
        ),
    )
    parser.add_argument(
        '--samples',
        type=int,
        default=20,
        help='Number of accepted views before auto-calibration (default: 20).',
    )
    parser.add_argument(
        '--min-motion',
        type=float,
        default=25.0,
        help=(
            'Minimum average detected-corner motion before accepting another '
            'automatic sample in pixels (default: 25).'
        ),
    )
    parser.add_argument(
        '--manual',
        action='store_true',
        help='Only accept samples when SPACE is pressed.',
    )
    parser.add_argument(
        '--output',
        type=Path,
        default=Path('camera_intrinsics.yaml'),
        help='Path for OpenCV YAML calibration output.',
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


def make_object_points(columns, rows, square_size):
    """Create the 3D chessboard model for one captured view."""
    object_points = np.zeros((rows * columns, 3), np.float32)
    object_points[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2)
    object_points *= square_size
    return object_points


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


def is_distinct_view(corners, previous_corners, min_motion):
    """Check whether the latest view moved enough to be useful."""
    if previous_corners is None:
        return True
    motion = np.linalg.norm(corners.reshape(-1, 2) - previous_corners, axis=1)
    return float(np.mean(motion)) >= min_motion


def put_text(frame, lines, colors=None):
    """Draw readable status text on the frame."""
    if colors is None:
        colors = [(255, 255, 255)] * len(lines)
    for index, (line, color) in enumerate(zip(lines, colors)):
        origin = (12, 30 + index * 28)
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
            color,
            1,
            cv2.LINE_AA,
        )


def calibrate(object_points, image_points, image_size):
    """Run OpenCV camera calibration and return result data."""
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    errors = []
    for index, model_points in enumerate(object_points):
        projected, _jacobian = cv2.projectPoints(
            model_points,
            rvecs[index],
            tvecs[index],
            camera_matrix,
            distortion,
        )
        error = cv2.norm(image_points[index], projected, cv2.NORM_L2)
        errors.append(error / len(projected))
    return rms, camera_matrix, distortion, float(np.mean(errors))


def save_parameters(path, image_size, rms, mean_error, camera_matrix, distortion):
    """Write camera intrinsics and metadata in OpenCV YAML format."""
    path.parent.mkdir(parents=True, exist_ok=True)
    storage = cv2.FileStorage(str(path), cv2.FILE_STORAGE_WRITE)
    if not storage.isOpened():
        raise SystemExit(f'Unable to write calibration file: {path}')
    try:
        storage.write('image_width', int(image_size[0]))
        storage.write('image_height', int(image_size[1]))
        storage.write('rms_reprojection_error', float(rms))
        storage.write('mean_reprojection_error', float(mean_error))
        storage.write('camera_matrix', camera_matrix)
        storage.write('distortion_coefficients', distortion)
    finally:
        storage.release()


def main():
    """Collect chessboard views, calibrate, and save intrinsics."""
    args = parse_args()
    pattern_size = (args.columns, args.rows)
    object_template = make_object_points(
        args.columns,
        args.rows,
        args.square_size,
    )
    cap = open_camera(args)

    object_points = []
    image_points = []
    previous_corners = None
    last_message = ""
    flash_timer = 0

    print(
        f"Looking for {args.columns}x{args.rows} inner corners. "
        "Move/tilt the chessboard between captures."
    )
    print("SPACE: capture manually   C: calibrate now   R: reset   Q/Esc: quit")

    try:
        while True:
            received, frame = cap.read()
            if not received:
                raise SystemExit("Camera stream stopped returning frames.")

            image_size = (frame.shape[1], frame.shape[0])
            found, corners = find_chessboard(frame, pattern_size)
            preview = frame.copy()

            # Draw coverage map of all previously accepted corners
            for points in image_points:
                for pt in points.reshape(-1, 2):
                    cv2.circle(preview, tuple(pt.astype(int)), 2, (0, 255, 0), -1)

            if found:
                cv2.drawChessboardCorners(preview, pattern_size, corners, found)

            key = cv2.waitKey(1) & 0xFF
            should_capture = False
            if found and args.manual and key == ord(" "):
                should_capture = True
            elif found and not args.manual:
                should_capture = is_distinct_view(
                    corners,
                    previous_corners,
                    args.min_motion,
                )

            if should_capture:
                object_points.append(object_template.copy())
                image_points.append(corners.copy())
                previous_corners = corners.reshape(-1, 2).copy()
                last_message = f"accepted sample {len(image_points)}/{args.samples}"
                print(last_message)
                flash_timer = 5  # Flash for 5 frames

            if flash_timer > 0:
                preview[:] = 255  # White flash
                flash_timer -= 1

            ready = len(image_points) >= args.samples
            if key == ord("c") and len(image_points) >= 3:
                ready = True
            elif key == ord("r"):
                object_points.clear()
                image_points.clear()
                previous_corners = None
                last_message = "samples reset"
                print(last_message)
            elif key in (ord("q"), 27):
                break

            status = "FOUND" if found else "NOT FOUND"
            status_color = (0, 255, 0) if found else (0, 0, 255)
            ready = len(image_points) >= args.samples
            samples_color = (0, 255, 0) if ready else (255, 255, 255)

            # Draw compact status HUD
            put_text(
                preview,
                [
                    f"CHESSBOARD: {status}",
                    f"SAMPLES: {len(image_points)}/{args.samples}",
                    last_message,
                ],
                colors=[status_color, samples_color, (255, 255, 255)],
            )
            cv2.imshow(WINDOW_NAME, preview)

            if ready:
                if len(image_points) < 3:
                    last_message = 'need at least 3 samples'
                    continue
                rms, camera_matrix, distortion, mean_error = calibrate(
                    object_points,
                    image_points,
                    image_size,
                )
                save_parameters(
                    args.output,
                    image_size,
                    rms,
                    mean_error,
                    camera_matrix,
                    distortion,
                )
                print(f'Saved calibration to {args.output}')
                print(f'RMS reprojection error: {rms:.4f}')
                print(f'Mean reprojection error: {mean_error:.4f} px')
                print('Camera matrix:')
                print(camera_matrix)
                print('Distortion coefficients:')
                print(distortion.reshape(-1))
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
