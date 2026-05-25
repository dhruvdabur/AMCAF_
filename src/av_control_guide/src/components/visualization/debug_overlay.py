"""
debug_overlay.py

Compact controller-focused debugging overlay for path tracking simulations.
"""

from math import degrees


class ControllerDebugOverlay:
    """
    Draws live controller diagnostics in a compact text box.
    """

    def __init__(self, vehicle, course=None, color="black"):
        self.vehicle = vehicle
        self.course = course
        self.color = color
        self._last_steer_rad = 0.0
        self._steer_spike_count = 0
        self._solver_fail_count = 0
        self._large_cte_count = 0
        self._last_diag = {}

    def _safe_cross_track_error_m(self):
        if not self.course:
            return None
        try:
            state = self.vehicle.state
            target_index = self.course.search_nearest_point_index(state)
            _, error_lat_m, _ = self.course.calculate_lonlat_error(state, target_index)
            return float(error_lat_m)
        except Exception:
            return None

    def update(self, time_s):
        controller = self.vehicle.controller
        if not controller:
            self._last_diag = {}
            return

        if hasattr(controller, "get_diagnostics"):
            self._last_diag = controller.get_diagnostics() or {}
        else:
            self._last_diag = {
                "controller_type": type(controller).__name__,
                "target_steer_rad": float(getattr(controller, "get_target_steer_rad", lambda: 0.0)()),
                "target_accel_mps2": float(getattr(controller, "get_target_accel_mps2", lambda: 0.0)()),
                "target_yaw_rate_rps": float(getattr(controller, "get_target_yaw_rate_rps", lambda: 0.0)()),
            }

        steer = float(self._last_diag.get("target_steer_rad", 0.0))
        if abs(steer - self._last_steer_rad) > 0.15:
            self._steer_spike_count += 1
        self._last_steer_rad = steer

        solver_success = self._last_diag.get("solver_success", True)
        if solver_success is False:
            self._solver_fail_count += 1

        cte = self._safe_cross_track_error_m()
        if cte is not None and abs(cte) > 2.0:
            self._large_cte_count += 1

    def _format_optional(self, value, fmt):
        if value is None:
            return "N/A"
        return fmt.format(value)

    def draw(self, axes, elems):
        controller_name = self._last_diag.get("controller_type", "N/A")
        steer_deg = degrees(float(self._last_diag.get("target_steer_rad", 0.0)))
        accel = float(self._last_diag.get("target_accel_mps2", 0.0))
        yaw_rate = float(self._last_diag.get("target_yaw_rate_rps", 0.0))
        speed = self._last_diag.get("target_speed_mps", None)
        cte = self._safe_cross_track_error_m()

        solver_ok = self._last_diag.get("solver_success", None)
        solver_state = "N/A" if solver_ok is None else ("OK" if solver_ok else "FAIL")
        solver_status = self._last_diag.get("solver_status", "N/A")
        solve_time = self._format_optional(self._last_diag.get("solve_time_ms"), "{0:.2f} ms")
        objective = self._format_optional(self._last_diag.get("objective_value"), "{0:.2f}")

        mppi_cost = self._last_diag.get("sample_cost_mean")
        ess = self._last_diag.get("effective_sample_size")
        ess_display = self._format_optional(ess, "{0:.1f}")

        metrics_lines = [
            "Debug Overlay ({0})".format(controller_name),
            "solver: {0} [{1}]".format(solver_state, solver_status),
            "solve time: {0}".format(solve_time),
            "objective: {0}".format(objective),
            "steer: {0:.2f} deg".format(steer_deg),
            "accel: {0:.2f} m/s^2".format(accel),
            "yaw rate: {0:.2f} rad/s".format(yaw_rate),
            "speed: {0}".format(self._format_optional(speed, "{0:.2f} m/s")),
            "cte: {0}".format(self._format_optional(cte, "{0:.2f} m")),
        ]

        if mppi_cost is not None:
            metrics_lines.append("sample mean cost: {0:.2f}".format(float(mppi_cost)))
            metrics_lines.append("effective samples: {0}".format(ess_display))

        metrics_lines.append(
            "events fail/spike/cte: {0}/{1}/{2}".format(
                self._solver_fail_count,
                self._steer_spike_count,
                self._large_cte_count,
            )
        )

        elems.append(
            axes.text(
                0.98,
                0.98,
                "\n".join(metrics_lines),
                transform=axes.transAxes,
                va="top",
                ha="right",
                fontsize=9,
                color=self.color,
                bbox={
                    "boxstyle": "round,pad=0.35",
                    "facecolor": "white",
                    "edgecolor": "0.7",
                    "alpha": 0.88,
                },
                zorder=11,
            )
        )
