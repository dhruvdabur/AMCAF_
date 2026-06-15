"""OpenCV drawing helpers for follower preview windows."""

import cv2
import numpy as np

OVERLAY_TEXT_SCALE = 1.0
OVERLAY_TEXT_VISIBLE = True


def noop(_value):
    """Opencv trackbar callback placeholder."""
    return None


def set_overlay_text_style(scale=None, visible=None):
    """Update global preview text scale and visibility."""
    global OVERLAY_TEXT_SCALE
    global OVERLAY_TEXT_VISIBLE
    if scale is not None:
        OVERLAY_TEXT_SCALE = max(0.1, min(float(scale), 3.0))
    if visible is not None:
        OVERLAY_TEXT_VISIBLE = bool(visible)


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
    if not OVERLAY_TEXT_VISIBLE:
        return
    lines = [
        f'MODE: {mode}  THR: {throttle}  ROLL: {roll}',
        (
            f'SPEED: {track_speed_pps:.1f}/{target_track_speed_pps:.1f} '
            f'PPS  CBF_SCALE: {cbf_scale:.2f}'
        ),
        f'VEL_ERR: {speed_error_pps:.1f} PPS',
        (
            f'MAE: {metrics["mean_abs_cte_px"]:.1f} PX  '
            f'RMSE: {metrics["rmse_cte_px"]:.1f} PX  '
            f'MAX: {metrics["max_abs_cte_px"]:.1f} PX'
        ),
        (
            f'HEAD: {metrics["mean_abs_heading_error_deg"]:.1f} DEG  '
            f'SPEED_ERR: {metrics["mean_abs_speed_error_pps"]:.1f} PPS'
        ),
        (
            f'STEER_EFF: {metrics["steering_effort_pwm_s"]:.0f}  '
            f'THR_EFF: {metrics["throttle_effort_pwm_s"]:.0f}'
        ),
        (
            f'CLEAR: {metrics["min_obstacle_clearance_px"]:.1f} PX  '
            f'COLL: {metrics["collision_samples"]}  '
            f'CBF: {metrics["cbf_interventions"]}  '
            f'LAPS: {metrics["laps_completed"]}'
        ),
        (
            f'LAP_LIMIT: {"ON" if lap_limit_enabled else "OFF"}  '
            f'TARGET: {target_laps}'
        ),
    ]
    if np.isfinite(nearest_static_clearance_px):
        clearance_text = f'{nearest_static_clearance_px:.1f} PX'
    else:
        clearance_text = 'INF'
    lines.append(f'STATIC_CLEAR: {clearance_text}')
    if cbf_qp_status != 'unused':
        lines.append(
            f'QP: {cbf_qp_status.upper()}  ACCEL: {cbf_qp_accel:.2f}  '
            f'DELTA: {cbf_qp_delta:.2f}'
        )
    scale = 0.7 * OVERLAY_TEXT_SCALE
    line_step = max(12, int(round(28 * OVERLAY_TEXT_SCALE)))
    y_start = max(14, int(round(30 * OVERLAY_TEXT_SCALE)))
    halo_thickness = max(1, int(round(4 * OVERLAY_TEXT_SCALE)))
    text_thickness = max(1, int(round(OVERLAY_TEXT_SCALE)))
    for index, line in enumerate(lines):
        origin = (12, y_start + index * line_step)
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (0, 0, 0),
            halo_thickness,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            scale,
            (255, 255, 255),
            text_thickness,
            cv2.LINE_AA,
        )


def draw_label(frame, origin, text, color, scale=0.5):
    """Draw readable debug text with a dark halo."""
    if not OVERLAY_TEXT_VISIBLE:
        return
    scaled = max(0.1, float(scale) * OVERLAY_TEXT_SCALE)
    halo_thickness = max(1, int(round(3 * OVERLAY_TEXT_SCALE)))
    text_thickness = max(1, int(round(OVERLAY_TEXT_SCALE)))
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scaled,
        (0, 0, 0),
        halo_thickness,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        text,
        origin,
        cv2.FONT_HERSHEY_SIMPLEX,
        scaled,
        color,
        text_thickness,
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
