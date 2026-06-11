"""OpenCV drawing helpers for follower preview windows."""

import cv2
import numpy as np


def noop(_value):
    """Opencv trackbar callback placeholder."""
    return None


def resize_for_preview(frame, target_width):
    """Resize preview output while preserving aspect ratio."""
    if target_width <= 0:
        return frame
    height, width = frame.shape[:2]
    if width <= target_width:
        return frame
    scale = target_width / float(width)
    return cv2.resize(
        frame,
        (target_width, int(round(height * scale))),
        interpolation=cv2.INTER_AREA,
    )


def put_status(
    frame,
    mode,
    throttle,
    roll,
    fresh,
    track_speed_pps,
    target_track_speed_pps,
    speed_error_pps,
    velocity_delta_pwm,
    cbf_scale,
    metrics,
    lap_limit_enabled,
    target_laps,
    nearest_static_clearance_px=float('inf'),
    cbf_qp_status='unused',
    cbf_qp_accel=0.0,
    cbf_qp_delta=0.0,
):
    """Draw controller status text onto a preview frame."""
    lines = [
        f'mode={mode} throttle={throttle} roll={roll}',
        f'command={"fresh" if fresh else "neutral"}',
        (
            f'track_speed={track_speed_pps:.1f}/{target_track_speed_pps:.1f} '
            f'pps cbf_scale={cbf_scale:.2f}'
        ),
        f'vel_err={speed_error_pps:.1f}pps vel_delta={velocity_delta_pwm:.1f}pwm',
        (
            f'MAE={metrics["mean_abs_cte_px"]:.1f}px '
            f'RMSE={metrics["rmse_cte_px"]:.1f}px '
            f'MAX={metrics["max_abs_cte_px"]:.1f}px'
        ),
        (
            f'head={metrics["mean_abs_heading_error_deg"]:.1f}deg '
            f'speed_err={metrics["mean_abs_speed_error_pps"]:.1f}pps'
        ),
        (
            f'steer_eff={metrics["steering_effort_pwm_s"]:.0f} '
            f'thr_eff={metrics["throttle_effort_pwm_s"]:.0f}'
        ),
        (
            f'clear={metrics["min_obstacle_clearance_px"]:.1f}px '
            f'coll={metrics["collision_samples"]} '
            f'cbf={metrics["cbf_interventions"]} '
            f'laps={metrics["laps_completed"]}'
        ),
        (
            f'lap_limit={"on" if lap_limit_enabled else "off"} '
            f'target={target_laps}'
        ),
    ]
    if np.isfinite(nearest_static_clearance_px):
        clearance_text = f'{nearest_static_clearance_px:.1f}px'
    else:
        clearance_text = 'inf'
    lines.append(f'static_clear={clearance_text}')
    if cbf_qp_status != 'unused':
        lines.append(
            f'qp={cbf_qp_status} accel={cbf_qp_accel:.2f} '
            f'delta={cbf_qp_delta:.2f}'
        )
    for index, line in enumerate(lines):
        origin = (12, 30 + index * 28)
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            4,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def draw_label(frame, origin, text, color, scale=0.5):
    """Draw readable debug text with a dark halo."""
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        3,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        1,
        cv2.LINE_AA,
    )


def draw_vector(frame, origin, vector, length, color, label=None):
    """Draw a vector arrow from an image-space origin."""
    start = np.array(origin, dtype=np.float32)
    direction = np.array(vector, dtype=np.float32)
    norm = np.linalg.norm(direction)
    if norm <= 1e-6:
        return
    end = start + direction / norm * length
    cv2.arrowedLine(
        frame,
        tuple(start.astype(int)),
        tuple(end.astype(int)),
        color,
        2,
        cv2.LINE_AA,
        tipLength=0.22,
    )
    if label:
        draw_label(
            frame,
            tuple(end.astype(int) + np.array([6, -6])),
            label,
            color,
        )
