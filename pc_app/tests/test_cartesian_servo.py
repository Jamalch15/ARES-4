import numpy as np
from pytest import approx

from app.cartesian_jog_debug import simulate_cartesian_jog
from app.cartesian_servo import CartesianServo, CartesianServoLimits
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.kinematics import _numeric_jacobian, geometric_task_jacobian


def test_geometric_jacobian_matches_endpoint_ik_numeric_model():
    config = load_config(EXAMPLE_CONFIG_PATH)

    for pose in (
        config.home_pose,
        [10.0, 45.0, 20.0, -30.0],
        [-35.0, 70.0, -40.0, 25.0],
    ):
        analytic = geometric_task_jacobian(pose, config.links)
        numeric = _numeric_jacobian({}, pose, config.links)

        assert analytic == approx(numeric, abs=0.005)


def test_opposite_axis_commands_have_symmetric_authority():
    config = load_config(EXAMPLE_CONFIG_PATH)
    start = [0.0, 25.0, 80.0, -50.0]

    for positive, negative in (
        ([40.0, 0.0, 0.0], [-40.0, 0.0, 0.0]),
        ([0.0, 40.0, 0.0], [0.0, -40.0, 0.0]),
        ([0.0, 0.0, 40.0], [0.0, 0.0, -40.0]),
    ):
        forward = simulate_cartesian_jog(config, start, positive, steps=20, dt_s=1.0 / 30.0)
        reverse = simulate_cartesian_jog(config, start, negative, steps=20, dt_s=1.0 / 30.0)

        assert forward["blocked_steps"] == 0
        assert reverse["blocked_steps"] == 0
        assert forward["metrics"]["progress_mm"] == approx(
            reverse["metrics"]["progress_mm"],
            abs=0.01,
        )
        assert forward["metrics"]["backward_steps"] == 0
        assert reverse["metrics"]["backward_steps"] == 0


def test_held_axis_never_generates_lateral_or_backward_motion():
    config = load_config(EXAMPLE_CONFIG_PATH)
    servo = CartesianServo(config.links, config.joints, [0.0, 25.0, 80.0, -50.0])
    limits = CartesianServoLimits(
        [joint.max_speed_deg_s for joint in config.joints],
        tcp_accel_mm_s2=360.0,
        phi_accel_deg_s2=240.0,
    )
    servo.set_command([40.0, 0.0, 0.0, 0.0])
    previous_x = None

    for _ in range(30):
        result = servo.step(1.0 / 30.0, limits)
        assert not result["blocked"]
        assert result["position_alignment"] is None or result["position_alignment"] > 0.999
        assert result["position_lateral_mm"] < 0.01
        current_x = result["predicted_fk"]["x_mm"]
        if previous_x is not None:
            assert current_x >= previous_x - 1e-6
        previous_x = current_x


def test_reversal_decelerates_before_changing_direction():
    config = load_config(EXAMPLE_CONFIG_PATH)
    servo = CartesianServo(config.links, config.joints, [0.0, 25.0, 80.0, -50.0])
    limits = CartesianServoLimits(
        [joint.max_speed_deg_s for joint in config.joints],
        tcp_accel_mm_s2=240.0,
        phi_accel_deg_s2=180.0,
    )
    servo.set_command([40.0, 0.0, 0.0, 0.0])
    for _ in range(8):
        servo.step(1.0 / 30.0, limits)

    servo.set_command([-40.0, 0.0, 0.0, 0.0])
    applied_x = []
    lateral = []
    for _ in range(14):
        result = servo.step(1.0 / 30.0, limits)
        applied_x.append(result["applied_task_velocity"][0])
        lateral.append(result["position_lateral_mm"])

    assert all(
        next_value <= value + 1e-9
        for value, next_value in zip(applied_x, applied_x[1:])
    )
    assert applied_x[-1] < 0.0
    assert max(lateral) < 0.01


def test_joint_acceleration_limits_bound_live_servo_velocity_changes():
    config = load_config(EXAMPLE_CONFIG_PATH)
    servo = CartesianServo(config.links, config.joints, [0.0, 25.0, 80.0, -50.0])
    limits = CartesianServoLimits(
        joint_speed_deg_s=[joint.max_speed_deg_s for joint in config.joints],
        joint_accel_deg_s2=[6.0, 7.0, 8.0, 9.0],
        tcp_accel_mm_s2=10_000.0,
        phi_accel_deg_s2=10_000.0,
    )
    servo.set_command([0.0, 60.0, 0.0, 0.0])

    first = servo.step(0.05, limits)
    second = servo.step(0.05, limits)

    for velocity, acceleration in zip(
        first["joint_velocity_deg_s"],
        limits.joint_accel_deg_s2,
        strict=True,
    ):
        assert abs(velocity) <= acceleration * 0.05 + 1e-8
    for previous, velocity, acceleration in zip(
        first["joint_velocity_deg_s"],
        second["joint_velocity_deg_s"],
        limits.joint_accel_deg_s2,
        strict=True,
    ):
        assert abs(velocity - previous) <= acceleration * 0.05 + 1e-8


def test_adding_phi_command_during_translation_is_not_treated_as_reversal():
    config = load_config(EXAMPLE_CONFIG_PATH)
    servo = CartesianServo(config.links, config.joints, [0.0, 25.0, 80.0, -50.0])
    limits = CartesianServoLimits(
        joint_speed_deg_s=[joint.max_speed_deg_s for joint in config.joints],
        joint_accel_deg_s2=[120.0, 120.0, 180.0, 180.0],
    )
    servo.set_command([0.0, 30.0, 0.0, 0.0])
    for _ in range(5):
        servo.step(1.0 / 30.0, limits)

    servo.set_command([0.0, 30.0, 0.0, 20.0])
    result = servo.step(1.0 / 30.0, limits)

    assert result["applied_task_velocity"][3] > 0.0


def test_inward_cartesian_command_retains_authority_at_reference_pose():
    config = load_config(EXAMPLE_CONFIG_PATH)
    servo = CartesianServo(config.links, config.joints, config.home_pose)
    limits = CartesianServoLimits(
        [joint.max_speed_deg_s for joint in config.joints],
        joint_accel_deg_s2=[joint.max_accel_deg_s2 for joint in config.joints],
        tcp_accel_mm_s2=360.0,
        phi_accel_deg_s2=240.0,
    )
    servo.set_command([0.0, 0.0, -40.0, 0.0])

    first = servo.step(1.0 / 30.0, limits)

    assert not first["blocked"]
    assert first["solver_mode"] in {
        "direction_constrained",
        "endpoint_singularity_escape",
    }
    assert first["achieved_delta"]["z_mm"] < 0.0
    assert first["position_alignment"] > 0.999
