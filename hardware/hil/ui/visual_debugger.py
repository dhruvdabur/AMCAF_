"""Visual debugger and preview renderer for the elliptical follower node."""

import math
import cv2
import numpy as np
from .preview import draw_label
from .preview import draw_vector

class EllipseVisualDebugger:
    """Encapsulates all OpenCV preview drawings and diagnostic displays."""

    def __init__(self, node):
        self.node = node

    def draw_road_scene(self, preview):
        """Draw the road scene and static obstacle field."""
        if self.node.road_boundaries:
            for boundary in self.node.road_boundaries:
                cv2.polylines(
                    preview,
                    [boundary.astype(np.int32)],
                    self.node.closed_road_scene_enabled(),
                    (180, 180, 180),
                    2,
                    cv2.LINE_AA,
                )
        for obstacle in self.node.static_obstacles:
            cv2.fillConvexPoly(
                preview,
                obstacle['polygon'].astype(np.int32),
                (40, 40, 220),
            )
            cv2.polylines(
                preview,
                [obstacle['polygon'].astype(np.int32)],
                True,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
        if self.node.latest_free_space_target is not None:
            cv2.circle(
                preview,
                tuple(self.node.latest_free_space_target.astype(int)),
                6,
                (255, 255, 0),
                -1,
            )

    def draw_debug_visuals(self, preview, center, target, tangent):
        """Draw dense geometry and CBF debug overlays."""
        self.draw_progress_debug(preview)
        self.draw_marker_trail(preview)
        if center is not None:
            self.draw_heading_debug(preview, center)
            # ponytail: Natively draw active virtual lidar feedback by default
            self.draw_lidar_feedback(preview, center)
        is_dclf = (self.node.config.controller_mode == 'pid_velocity_dclf_dcbf')
        if not is_dclf and center is not None and target is not None:
            cv2.line(
                preview,
                tuple(center.astype(int)),
                tuple(target.astype(int)),
                (0, 180, 255),
                2,
                cv2.LINE_AA,
            )
            midpoint = ((center + target) * 0.5).astype(int)
            draw_label(
                preview,
                tuple(midpoint),
                f'cte={self.node.latest_lateral_error_px:.1f}px',
                (0, 180, 255),
            )
        if target is not None and tangent is not None:
            draw_vector(preview, target, tangent, 70.0, (0, 255, 255), 'target')
        if is_dclf:
            self.draw_dclf_dcbf_visuals(preview, center, target)
        else:
            self.draw_cbf_ellipse_debug(preview, center)
        if self.node.config.controller_mode == 'mpc_cbf':
            opt_traj = getattr(self.node.controller.mpc_controller, 'optimal_trajectory', None)
            if opt_traj is not None:
                px_pred, py_pred = opt_traj
                points = [tuple(map(int, [x, y])) for x, y in zip(px_pred, py_pred)]
                for i in range(len(points) - 1):
                    cv2.line(preview, points[i], points[i+1], (0, 255, 0), 2, cv2.LINE_AA)
        self.draw_controller_debug_panel(preview)
        self.draw_free_space_interval(preview)
        self.draw_safety_bars(preview)

    def draw_dclf_dcbf_visuals(self, preview, center, target):
        """Draw the DCLF-DCBF specific overlays."""
        if center is None:
            return
        
        # 1. Draw DCBF dashed neon ellipse
        color = (255, 0, 255) if self.node.cbf_active else (255, 255, 0)
        if self.node.cbf_qp_status == 'infeasible' or self.node.cbf_qp_h < 0.0:
            color = (0, 0, 255)
        a_ell_px, b_ell_px = self.node.controller.cbf_ellipse_axes_px()
        a_ell_cm, b_ell_cm = self.node.controller.cbf_ellipse_axes_cm()
        marker_scale_px = self.node.controller.cbf_ellipse_marker_scale_px()
        heading_rad = float(self.node.last_marker_heading)
        
        # ponytail: Use native cv2.ellipse platform feature instead of custom 36-segment loop
        axes = (max(1, int(round(a_ell_px))), max(1, int(round(b_ell_px))))
        cv2.ellipse(
            preview,
            tuple(center.astype(int)),
            axes,
            math.degrees(heading_rad),
            0.0,
            360.0,
            color,
            2,
            cv2.LINE_AA,
        )
                
        label_origin = center + np.array([a_ell_px + 8.0, b_ell_px + 12.0])
        draw_label(
            preview,
            tuple(label_origin.astype(int)),
            (
                f'DCBF a={self.node.config.cbf_a_ell:.2f} b={self.node.config.cbf_b_ell:.2f} '
                f'scale={marker_scale_px:.1f}px -> '
                f'{a_ell_px:.0f}x{b_ell_px:.0f}px '
                f'({a_ell_cm:.0f}x{b_ell_cm:.0f}cm)'
            ),
            color,
            scale=0.42,
        )
        
        # 2. Draw DCLF Lyapunov convergence corridor
        if target is not None:
            overlay = preview.copy()
            for r_factor in [1.0, 2.0, 3.0]:
                cv2.ellipse(
                    overlay,
                    tuple(target.astype(int)),
                    (int(r_factor * 12), int(r_factor * 8)),
                    math.degrees(heading_rad),
                    0.0,
                    360.0,
                    (0, 255, 120),
                    1,
                    cv2.LINE_AA,
                )
            cv2.addWeighted(overlay, 0.4, preview, 0.6, 0.0, preview)
            
            cv2.line(
                preview,
                tuple(center.astype(int)),
                tuple(target.astype(int)),
                (0, 255, 120),
                2,
                cv2.LINE_AA,
            )
            midpoint = ((center + target) * 0.5).astype(int)
            draw_label(
                preview,
                tuple(midpoint + np.array([0, -10])),
                f'DCLF cte={self.node.latest_lateral_error_px:.1f}px',
                (0, 255, 120),
            )

    def draw_progress_debug(self, preview):
        """Annotate nearest and lookahead points on the track."""
        if self.node.track_points is None:
            return
        # ponytail: Natively connect lookahead path segment using simple line segments
        if self.node.latest_nearest_index is not None and self.node.latest_target_index is not None:
            idx_start = self.node.latest_nearest_index
            idx_end = self.node.latest_target_index
            path_len = len(self.node.track_points)
            pts = [
                tuple(self.node.track_points[i % path_len].astype(int))
                for i in range(idx_start, idx_end + 1)
            ]
            for i in range(len(pts) - 1):
                cv2.line(preview, pts[i], pts[i + 1], (0, 255, 255), 2, cv2.LINE_AA)
        if self.node.latest_nearest_index is not None:
            nearest = self.node.track_points[self.node.latest_nearest_index]
            cv2.circle(preview, tuple(nearest.astype(int)), 5, (255, 255, 255), -1)
            draw_label(
                preview,
                tuple(nearest.astype(int) + np.array([8, -8])),
                f'n={self.node.latest_nearest_index}',
                (255, 255, 255),
            )
        if self.node.latest_target_index is not None:
            target_center = self.node.track_points[self.node.latest_target_index]
            cv2.circle(
                preview,
                tuple(target_center.astype(int)),
                5,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )
            draw_label(
                preview,
                tuple(target_center.astype(int) + np.array([8, 14])),
                f'la={self.node.config.lookahead_points}',
                (0, 255, 255),
            )

    def draw_marker_trail(self, preview):
        """Draw recent marker centers with fading intensity."""
        if len(self.node.marker_trail) < 2:
            return
        count = len(self.node.marker_trail)
        for index in range(1, count):
            ratio = index / float(max(1, count - 1))
            color = (int(60 + 120 * ratio), int(80 + 120 * ratio), 255)
            cv2.line(
                preview,
                tuple(self.node.marker_trail[index - 1].astype(int)),
                tuple(self.node.marker_trail[index].astype(int)),
                color,
                2,
                cv2.LINE_AA,
            )

    def draw_heading_debug(self, preview, center):
        """Draw marker heading and signed vehicle-right steering axis."""
        heading_vec = np.array(
            [math.cos(self.node.last_marker_heading), math.sin(self.node.last_marker_heading)],
            dtype=np.float32,
        )
        right_vec = np.array(
            [-math.sin(self.node.last_marker_heading), math.cos(self.node.last_marker_heading)],
            dtype=np.float32,
        )
        draw_vector(preview, center, heading_vec, 70.0, (0, 255, 0), 'heading')
        draw_vector(preview, center, right_vec, 55.0, (255, 170, 0), 'right')
        heading_deg = math.degrees(
            self.node.display_heading_error_rad(self.node.latest_heading_error_rad)
        )
        draw_label(
            preview,
            tuple(center.astype(int) + np.array([12, 24])),
            f'head_err={heading_deg:.1f}deg',
            (0, 255, 0),
        )

    def draw_cbf_ellipse_debug(self, preview, center):
        """Draw the marker-scaled QP CBF ellipse around the car."""
        if center is None:
            return
        color = (0, 165, 255) if self.node.cbf_active else (120, 170, 220)
        a_ell_px, b_ell_px = self.node.controller.cbf_ellipse_axes_px()
        a_ell_cm, b_ell_cm = self.node.controller.cbf_ellipse_axes_cm()
        marker_scale_px = self.node.controller.cbf_ellipse_marker_scale_px()
        angle_deg = math.degrees(float(self.node.last_marker_heading))
        axes = (
            max(1, int(round(a_ell_px))),
            max(1, int(round(b_ell_px))),
        )
        cv2.ellipse(
            preview,
            tuple(center.astype(int)),
            axes,
            angle_deg,
            0.0,
            360.0,
            color,
            2,
            cv2.LINE_AA,
        )
        label_origin = center + np.array([a_ell_px + 8.0, b_ell_px + 12.0])
        draw_label(
            preview,
            tuple(label_origin.astype(int)),
            (
                f'QP a={self.node.config.cbf_a_ell:.2f} b={self.node.config.cbf_b_ell:.2f} '
                f'scale={marker_scale_px:.1f}px -> '
                f'{a_ell_px:.0f}x{b_ell_px:.0f}px '
                f'({a_ell_cm:.0f}x{b_ell_cm:.0f}cm)'
            ),
            color,
            scale=0.42,
        )

    def draw_free_space_interval(self, preview):
        """Draw the selected lateral free interval at the lookahead point."""
        if (
            self.node.latest_target_index is None
            or self.node.latest_free_space_interval is None
            or self.node.road_normals is None
        ):
            return
        center = self.node.track_points[self.node.latest_target_index]
        normal = self.node.road_normals[self.node.latest_target_index]
        lower, upper = self.node.latest_free_space_interval
        start = center + normal * lower
        end = center + normal * upper
        cv2.line(
            preview,
            tuple(start.astype(int)),
            tuple(end.astype(int)),
            (255, 255, 0),
            4,
            cv2.LINE_AA,
        )
        draw_label(
            preview,
            tuple(((start + end) * 0.5).astype(int) + np.array([8, -10])),
            f'free={upper - lower:.0f}px',
            (255, 255, 0),
        )

    def draw_safety_bars(self, preview):
        """Draw compact CBF clearance bars in the lower-left corner."""
        if not self.node.latest_safety_clearances:
            return
        height, _width = preview.shape[:2]
        labels = ('edge', 'lateral', 'heading', 'obstacle', 'min')
        x = 12
        y = max(24, height - 28 * len(labels) - 12)
        max_clearance = max(1.0, self.node.cbf_stop_error_px)
        for index, label in enumerate(labels):
            value = self.node.latest_safety_clearances.get(label, float('inf'))
            bar_y = y + index * 28
            if math.isfinite(value):
                ratio = max(0.0, min((value + 30.0) / (max_clearance + 30.0), 1.0))
                color = (0, 220, 0) if value >= 0.0 else (0, 0, 255)
                text_value = f'{value:.0f}'
            else:
                ratio = 1.0
                color = (180, 180, 180)
                text_value = 'inf'
            cv2.rectangle(preview, (x, bar_y), (x + 150, bar_y + 16), (20, 20, 20), -1)
            cv2.rectangle(
                preview,
                (x, bar_y),
                (x + int(round(150 * ratio)), bar_y + 16),
                color,
                -1,
            )
            draw_label(
                preview,
                (x + 160, bar_y + 13),
                f'{label}:{text_value}',
                color,
                scale=0.48,
            )

    def draw_controller_debug_panel(self, preview):
        """Draw compact CBF/QP diagnostics."""
        height, width = preview.shape[:2]
        panel_width = 300
        panel_height = 172
        x = max(12, width - panel_width - 12)
        y = 12
        overlay = preview.copy()
        cv2.rectangle(
            overlay,
            (x, y),
            (x + panel_width, y + panel_height),
            (25, 25, 25),
            -1,
        )
        cv2.addWeighted(overlay, 0.55, preview, 0.45, 0.0, preview)
        cv2.rectangle(
            preview,
            (x, y),
            (x + panel_width, y + panel_height),
            self.cbf_debug_color(),
            1,
            cv2.LINE_AA,
        )
        margin = self.cbf_qp_constraint_margin()
        margin_text = 'n/a' if margin is None else f'{margin:.3f}'
        clear_text = (
            'inf'
            if not math.isfinite(self.node.nearest_static_clearance_px)
            else f'{self.node.nearest_static_clearance_px:.1f}px'
        )
        if self.node.config.controller_mode == 'pid_velocity_dclf_dcbf':
            mode_title = 'DCLF-DCBF'
        elif self.node.config.controller_mode == 'mpc_cbf':
            mode_title = 'MPC-CBF'
        else:
            mode_title = 'CBF-QP'
        lines = [
            f'{mode_title} {self.node.cbf_qp_status}',
            f'h {self.node.cbf_qp_h:.3f}  hd {self.node.cbf_qp_h_dot:.3f}  hdd {self.node.cbf_qp_h_ddot:.3f}',
            f'margin {margin_text}  rhs {self.node.cbf_qp_rhs:.3f}',
            f'a {self.node.cbf_qp_accel:.2f}  d {self.node.cbf_qp_delta:.2f}',
            f'slack {self.node.cbf_qp_slack:.3f}  clear {clear_text}',
            f'COLL {self.node.metrics.collision_samples}  {mode_title} {self.node.metrics.cbf_interventions}',
        ]
        for index, line in enumerate(lines):
            draw_label(
                preview,
                (x + 10, y + 24 + index * 22),
                line,
                (255, 255, 255),
                scale=0.46,
            )
        self.draw_horizontal_meter(
            preview,
            x + 10,
            y + panel_height - 18,
            panel_width - 20,
            self.cbf_debug_meter_ratio(),
            self.cbf_debug_color(),
        )

    def cbf_qp_constraint_margin(self):
        """Return lhs-rhs for the displayed closest-obstacle CBF constraint."""
        if self.node.cbf_qp_status == 'unused':
            return None
        lhs = (
            self.node.cbf_qp_lhs_a * self.node.cbf_qp_accel
            + self.node.cbf_qp_lhs_delta * self.node.cbf_qp_delta
            + self.node.cbf_qp_slack
        )
        return lhs - self.node.cbf_qp_rhs

    def cbf_debug_color(self):
        """Return panel color for current CBF state."""
        margin = self.cbf_qp_constraint_margin()
        if self.node.cbf_qp_status == 'infeasible' or self.node.cbf_qp_h < 0.0:
            return (0, 0, 255)
        if margin is not None and margin < 0.0:
            return (0, 140, 255)
        if self.node.cbf_active:
            return (0, 200, 255)
        return (0, 220, 0)

    def cbf_debug_meter_ratio(self):
        """Return a bounded health value for the CBF panel meter."""
        if self.node.cbf_qp_status == 'unused':
            return 1.0
        margin = self.cbf_qp_constraint_margin()
        if margin is None:
            return 1.0
        return max(0.0, min((float(margin) + 1.0) / 2.0, 1.0))

    @staticmethod
    def draw_horizontal_meter(preview, x, y, width, ratio, color):
        """Draw a bounded 0..1 meter used by debug panels."""
        bounded_ratio = max(0.0, min(float(ratio), 1.0))
        cv2.rectangle(preview, (x, y), (x + width, y + 8), (70, 70, 70), -1)
        cv2.rectangle(
            preview,
            (x, y),
            (x + int(round(width * bounded_ratio)), y + 8),
            color,
            -1,
        )
        cv2.rectangle(preview, (x, y), (x + width, y + 8), (230, 230, 230), 1)

    def draw_lidar_feedback(self, preview, center):
        """Draw virtual lidar returns from the marker/car center."""
        if not self.node.latest_lidar_points:
            return
        origin = tuple(center.astype(int))
        for point in self.node.latest_lidar_points:
            hit = (int(round(point.x_px)), int(round(point.y_px)))
            cv2.line(preview, origin, hit, (255, 0, 255), 1, cv2.LINE_AA)
            cv2.circle(preview, hit, 3, (255, 0, 255), -1)

        critical_x = getattr(self.node.controller, 'cbf_qp_obstacle_x', None)
        critical_y = getattr(self.node.controller, 'cbf_qp_obstacle_y', None)
        if critical_x is not None and critical_y is not None:
            crit_hit = (int(round(critical_x)), int(round(critical_y)))
            # Draw a thicker yellow line to the critical point
            cv2.line(preview, origin, crit_hit, (0, 255, 255), 2, cv2.LINE_AA)
            # Draw a larger yellow circle at the critical point
            cv2.circle(preview, crit_hit, 6, (0, 255, 255), -1)
            # Draw an outer red ring to highlight it
            cv2.circle(preview, crit_hit, 8, (0, 0, 255), 1, cv2.LINE_AA)
            # Add a small text label
            draw_label(preview, (crit_hit[0] + 10, crit_hit[1] - 5), "x_obs", (0, 255, 255), scale=0.45)
