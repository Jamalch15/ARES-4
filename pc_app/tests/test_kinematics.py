from pytest import approx

from app.config import LinkConfig, load_config
from app.kinematics import forward_kinematics, inverse_kinematics


def test_forward_kinematics_zero_pose_extends_up_from_shoulder():
    links = LinkConfig(
        base_height_mm=80.0,
        upper_arm_mm=140.0,
        forearm_mm=120.0,
        wrist_mm=55.0,
        tool_mm=35.0,
    )

    result = forward_kinematics([0.0, 0.0, 0.0, 0.0], links)

    assert abs(result["x_mm"]) < 1e-9
    assert abs(result["y_mm"]) < 1e-9
    assert result["z_mm"] == 430.0
    assert result["tool_phi_deg"] == 90.0


def test_forward_kinematics_base_zero_points_along_y_axis():
    links = LinkConfig(0.0, 100.0, 0.0, 0.0, 0.0)

    result = forward_kinematics([0.0, 90.0, 0.0, 0.0], links)

    assert abs(result["x_mm"]) < 1e-9
    assert round(result["y_mm"], 6) == 100.0
    assert abs(result["z_mm"]) < 1e-9


def test_forward_kinematics_base_yaw_rotates_positive_toward_negative_x():
    links = LinkConfig(0.0, 100.0, 0.0, 0.0, 0.0)

    result = forward_kinematics([90.0, 90.0, 0.0, 0.0], links)

    assert round(result["x_mm"], 6) == -100.0
    assert abs(result["y_mm"]) < 1e-9


def test_inverse_kinematics_round_trips_reachable_target():
    config = load_config()
    original = [15.0, 35.0, 30.0, -20.0]
    target_fk = forward_kinematics(original, config.links)

    result = inverse_kinematics(
        {
            "x_mm": target_fk["x_mm"],
            "y_mm": target_fk["y_mm"],
            "z_mm": target_fk["z_mm"],
            "phi_deg": target_fk["tool_phi_deg"],
        },
        config.links,
        config.joints,
        original,
    )

    assert result["ok"]
    selected_fk = result["selected"]["fk"]
    assert selected_fk["x_mm"] == approx(target_fk["x_mm"], abs=1e-6)
    assert selected_fk["y_mm"] == approx(target_fk["y_mm"], abs=1e-6)
    assert selected_fk["z_mm"] == approx(target_fk["z_mm"], abs=1e-6)
    assert selected_fk["tool_phi_deg"] == approx(target_fk["tool_phi_deg"], abs=1e-6)


def test_inverse_kinematics_returns_elbow_branches():
    config = load_config()
    target = {"x_mm": -80.0, "y_mm": 180.0, "z_mm": 190.0, "phi_deg": 0.0}

    result = inverse_kinematics(target, config.links, config.joints, config.home_pose)

    branches = {candidate["branch"] for candidate in result["candidates"]}
    assert branches == {"elbow_up", "elbow_down"}


def test_inverse_kinematics_rejects_unreachable_target():
    config = load_config()

    result = inverse_kinematics(
        {"x_mm": 2000.0, "y_mm": 0.0, "z_mm": 2000.0, "phi_deg": 0.0},
        config.links,
        config.joints,
        config.home_pose,
    )

    assert not result["ok"]
    assert "unreachable" in result["notes"][0]


def test_inverse_kinematics_filters_joint_limits():
    config = load_config()
    target_fk = forward_kinematics([180.0, 60.0, -40.0, 10.0], config.links)

    result = inverse_kinematics(
        {
            "x_mm": target_fk["x_mm"],
            "y_mm": target_fk["y_mm"],
            "z_mm": target_fk["z_mm"],
            "phi_deg": target_fk["tool_phi_deg"],
        },
        config.links,
        config.joints,
        config.home_pose,
    )

    assert not result["ok"]
    assert any("base" in reason for candidate in result["candidates"] for reason in candidate["reasons"])


def test_inverse_kinematics_prefers_nearest_valid_solution():
    config = load_config()
    target_fk = forward_kinematics([-90.0, -20.0, -80.0, 80.0], config.links)
    target = {
        "x_mm": target_fk["x_mm"],
        "y_mm": target_fk["y_mm"],
        "z_mm": target_fk["z_mm"],
        "phi_deg": target_fk["tool_phi_deg"],
    }
    first = inverse_kinematics(target, config.links, config.joints, config.home_pose)
    valid = [candidate for candidate in first["candidates"] if candidate["valid"]]
    assert len(valid) >= 2

    expected = valid[-1]
    second = inverse_kinematics(target, config.links, config.joints, expected["angles_deg"])

    assert second["selected_branch"] == expected["branch"]
