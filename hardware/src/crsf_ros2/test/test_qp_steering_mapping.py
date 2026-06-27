from types import SimpleNamespace

import pytest

from crsf_ros2.hil.control.ellipse_static_controller import EllipseStaticController


def make_controller():
    controller = EllipseStaticController.__new__(EllipseStaticController)
    controller.config = SimpleNamespace(
        right_pwm=1150,
        center_steering_pwm=1500,
        left_pwm=1850,
        qp_min_delta=-0.8,
        qp_max_delta=0.8,
    )
    return controller


def test_qp_positive_delta_maps_to_right_pwm():
    controller = make_controller()

    assert controller.roll_pwm_to_qp_delta(1150) == pytest.approx(0.8)
    assert controller.qp_delta_to_roll_pwm(0.8) == pytest.approx(1150)


def test_qp_negative_delta_maps_to_left_pwm():
    controller = make_controller()

    assert controller.roll_pwm_to_qp_delta(1850) == pytest.approx(-0.8)
    assert controller.qp_delta_to_roll_pwm(-0.8) == pytest.approx(1850)


def test_qp_center_delta_maps_to_center_pwm():
    controller = make_controller()

    assert controller.roll_pwm_to_qp_delta(1500) == pytest.approx(0.0)
    assert controller.qp_delta_to_roll_pwm(0.0) == pytest.approx(1500)
