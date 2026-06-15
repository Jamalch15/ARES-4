from __future__ import annotations

from copy import deepcopy
from typing import Any

from .config import RobotConfig
from .kinematics import forward_kinematics, inverse_kinematics
from .safety import validate_joint_targets


DEFAULT_COLOR_PROFILES: dict[str, dict[str, Any]] = {
    "red": {
        "enabled": True,
        "hsv_min": [0, 80, 60],
        "hsv_max": [12, 255, 255],
        "min_area_px": 250,
        "drop_zone": "dropoff_a",
    },
    "blue": {
        "enabled": True,
        "hsv_min": [95, 80, 50],
        "hsv_max": [130, 255, 255],
        "min_area_px": 250,
        "drop_zone": "dropoff_b",
    },
}


def default_named_positions(config: RobotConfig) -> dict[str, dict[str, Any]]:
    fk = forward_kinematics(config.home_pose, config.links)
    safe = config.home_pose.copy()
    if len(safe) >= 2:
        safe[1] = max(config.joints[1].min_deg, min(config.joints[1].max_deg, 35.0))
    return {
        "home": {"type": "joint", "angles_deg": config.home_pose},
        "safe": {"type": "joint", "angles_deg": safe},
        "pickup_test": {
            "type": "cartesian",
            "target": {
                "x_mm": fk["x_mm"],
                "y_mm": max(120.0, fk["y_mm"]),
                "z_mm": max(35.0, fk["z_mm"] - 120.0),
                "phi_deg": fk["tool_phi_deg"],
            },
        },
        "dropoff_a": {
            "type": "cartesian",
            "target": {"x_mm": -120.0, "y_mm": 180.0, "z_mm": 45.0, "phi_deg": 0.0},
        },
        "dropoff_b": {
            "type": "cartesian",
            "target": {"x_mm": 120.0, "y_mm": 180.0, "z_mm": 45.0, "phi_deg": 0.0},
        },
    }


def named_positions(config: RobotConfig) -> dict[str, dict[str, Any]]:
    raw = config.raw.get("named_positions")
    if isinstance(raw, dict) and raw:
        merged = default_named_positions(config)
        merged.update(deepcopy(raw))
        return merged
    return default_named_positions(config)


def camera_settings(config: RobotConfig) -> dict[str, Any]:
    defaults = {
        "source_index": 0,
        "enabled": False,
        "calibration": {
            "image_points": [],
            "robot_points": [],
        },
    }
    raw = config.raw.get("camera")
    if isinstance(raw, dict):
        defaults.update(deepcopy(raw))
    return defaults


def color_profiles(config: RobotConfig) -> dict[str, dict[str, Any]]:
    raw = config.raw.get("color_profiles")
    if isinstance(raw, dict) and raw:
        merged = deepcopy(DEFAULT_COLOR_PROFILES)
        merged.update(deepcopy(raw))
        return merged
    return deepcopy(DEFAULT_COLOR_PROFILES)


def drop_zones(config: RobotConfig) -> dict[str, dict[str, float]]:
    positions = named_positions(config)
    zones: dict[str, dict[str, float]] = {}
    for name in ["dropoff_a", "dropoff_b"]:
        value = positions.get(name, {})
        target = value.get("target") if isinstance(value.get("target"), dict) else value
        if isinstance(target, dict):
            zones[name] = {
                "x_mm": float(target.get("x_mm", target.get("x", 0.0))),
                "y_mm": float(target.get("y_mm", target.get("y", 0.0))),
                "z_mm": float(target.get("z_mm", target.get("z", 45.0))),
                "phi_deg": float(target.get("phi_deg", target.get("phi", 0.0))),
            }
    raw = config.raw.get("drop_zones")
    if isinstance(raw, dict):
        for name, value in raw.items():
            if isinstance(value, dict):
                zones[name] = {
                    "x_mm": float(value.get("x_mm", value.get("x", 0.0))),
                    "y_mm": float(value.get("y_mm", value.get("y", 0.0))),
                    "z_mm": float(value.get("z_mm", value.get("z", 45.0))),
                    "phi_deg": float(value.get("phi_deg", value.get("phi", 0.0))),
                }
    return zones


def tool_settings(config: RobotConfig) -> dict[str, Any]:
    defaults: dict[str, Any] = {
        "type": "servo_gripper",
        "open_value": 0.0,
        "closed_value": 1.0,
    }
    raw = config.raw.get("tool")
    if isinstance(raw, dict):
        defaults.update(deepcopy(raw))
    return defaults


def task_defaults(config: RobotConfig) -> dict[str, Any]:
    defaults = {
        "safe_position": "safe",
        "approach_height_mm": 80.0,
        "pickup_height_mm": 25.0,
        "dropoff_height_mm": 45.0,
        "default_drop_zone": "dropoff_a",
    }
    raw = config.raw.get("task_defaults")
    if isinstance(raw, dict):
        defaults.update(deepcopy(raw))
    return defaults


def validate_named_position(config: RobotConfig, name: str, position: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    kind = str(position.get("type") or position.get("kind") or "joint").lower()
    if kind == "joint":
        angles = position.get("angles_deg")
        if not isinstance(angles, list):
            return [f"{name} is missing angles_deg"]
        result = validate_joint_targets(config, [float(value) for value in angles])
        if not result.ok:
            errors.append(result.reason)
        return errors

    target = position.get("target") if isinstance(position.get("target"), dict) else position
    if not isinstance(target, dict):
        return [f"{name} is missing target"]
    pose = {
        "x_mm": float(target.get("x_mm", target.get("x", 0.0))),
        "y_mm": float(target.get("y_mm", target.get("y", 0.0))),
        "z_mm": float(target.get("z_mm", target.get("z", 0.0))),
        "phi_deg": float(target.get("phi_deg", target.get("phi", 0.0))),
    }
    ik = inverse_kinematics(pose, config.links, config.joints, config.home_pose)
    if not ik["ok"]:
        errors.append(f"{name} has no valid IK solution")
    return errors

