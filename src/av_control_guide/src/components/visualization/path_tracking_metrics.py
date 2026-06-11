"""
path_tracking_metrics.py

Live path-tracking performance metrics for X-Y simulations.
"""

from math import degrees


class PathTrackingMetrics:
    """
    Draws live path-tracking metrics on the plot.
    """

    def __init__(self, vehicle, course, color='black'):
        """
        Constructor
        vehicle: Vehicle object with state and controller members
        course: Course object used by the controller
        color: Text color
        """

        self.vehicle = vehicle
        self.course = course
        self.color = color
        self.sample_count = 0
        self.abs_cross_track_error_sum_m = 0.0
        self.steering_effort_rad_s = 0.0
        self.steering_angle_sum_rad = 0.0

    def _current_cross_track_error_m(self):
        state = self.vehicle.state
        target_index = self.course.search_nearest_point_index(state)
        _, error_lat_m, _ = self.course.calculate_lonlat_error(state, target_index)
        return error_lat_m

    def _current_steering_angle_rad(self):
        controller = self.vehicle.controller
        if controller and hasattr(controller, "get_target_steer_rad"):
            return controller.get_target_steer_rad()
        return 0.0

    def update(self, time_s):
        """
        Updates running metrics once per simulation frame.
        """

        cross_track_error_m = self._current_cross_track_error_m()
        steering_angle_rad = self._current_steering_angle_rad()

        self.sample_count += 1
        self.abs_cross_track_error_sum_m += abs(cross_track_error_m)
        self.steering_effort_rad_s += abs(steering_angle_rad) * time_s
        self.steering_angle_sum_rad += steering_angle_rad

    def mean_abs_cross_track_error_m(self):
        if self.sample_count == 0:
            return 0.0
        return self.abs_cross_track_error_sum_m / self.sample_count

    def mean_steering_angle_rad(self):
        if self.sample_count == 0:
            return 0.0
        return self.steering_angle_sum_rad / self.sample_count

    def draw(self, axes, elems):
        """
        Draws metric values in the top-left corner of the axes.
        """

        metrics_text = (
            "Mean abs CTE: {0:.3f} m\n"
            "Steering effort: {1:.3f} rad*s\n"
            "Mean steering: {2:.2f} deg"
        ).format(
            self.mean_abs_cross_track_error_m(),
            self.steering_effort_rad_s,
            degrees(self.mean_steering_angle_rad()),
        )

        elems.append(
            axes.text(
                0.02,
                0.98,
                metrics_text,
                transform=axes.transAxes,
                va='top',
                ha='left',
                fontsize=10,
                color=self.color,
                bbox={
                    'boxstyle': 'round,pad=0.35',
                    'facecolor': 'white',
                    'edgecolor': '0.7',
                    'alpha': 0.85,
                },
                zorder=10,
            )
        )
