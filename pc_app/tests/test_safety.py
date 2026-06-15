from app.config import load_config
from app.robot_state import MotionState, RobotState
from app.safety import validate_can_move, validate_joint_targets


def test_joint_limits_accept_home_pose():
    config = load_config()

    result = validate_joint_targets(config, config.home_pose)

    assert result.ok


def test_joint_limits_reject_out_of_range_target():
    config = load_config()
    targets = config.home_pose.copy()
    targets[0] = 999.0

    result = validate_joint_targets(config, targets)

    assert not result.ok
    assert "base" in result.reason


def test_estop_blocks_movement():
    config = load_config()
    state = RobotState(config.joint_names, config.home_pose, config.home_pose)
    state.motion_state = MotionState.ESTOP

    result = validate_can_move(state)

    assert not result.ok
    assert "emergency stop" in result.reason
