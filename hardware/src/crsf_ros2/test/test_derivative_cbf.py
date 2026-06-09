from crsf_ros2.hil.controllers.pid import PIDController
from crsf_ros2.hil.controllers.pid import PIDVelocityCBFController


def make_controller():
    return PIDVelocityCBFController(
        PIDController(kp=0.0, ki=0.0, kd=0.0, integral_limit=1.0)
    )


def apply(controller, lateral_error_px, now):
    return controller.apply_cbf(
        throttle_pwm=1600.0,
        neutral_throttle_pwm=1500.0,
        center_px=(320.0, 240.0),
        heading_rad=0.0,
        lateral_error_px=lateral_error_px,
        heading_error_rad=0.0,
        image_size=(640.0, 480.0),
        obstacles=[],
        slow_error_px=55.0,
        stop_error_px=120.0,
        stop_heading_rad=1.2,
        edge_margin_px=45.0,
        obstacle_margin_px=42.0,
        cbf_h_px=42.0,
        cbf_alpha=1.0,
        now=now,
    )


def test_derivative_cbf_scales_when_barrier_is_decreasing():
    controller = make_controller()

    first = apply(controller, lateral_error_px=0.0, now=1.0)
    second = apply(controller, lateral_error_px=80.0, now=2.0)

    assert first.scale == 1.0
    assert second.scale == 0.5
    assert second.throttle_pwm == 1550.0


def test_derivative_cbf_stops_when_barrier_is_violated():
    controller = make_controller()

    result = apply(controller, lateral_error_px=130.0, now=1.0)

    assert result.scale == 0.0
    assert result.throttle_pwm == 1500.0


def test_derivative_cbf_allows_throttle_when_barrier_is_recovering():
    controller = make_controller()

    apply(controller, lateral_error_px=80.0, now=1.0)
    result = apply(controller, lateral_error_px=40.0, now=2.0)

    assert result.scale == 1.0
    assert result.throttle_pwm == 1600.0
