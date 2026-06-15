from __future__ import annotations

from typing import Any

from .config import RobotConfig
from .demo_settings import drop_zones, named_positions, task_defaults


def _cartesian_target(raw: dict[str, Any]) -> dict[str, float]:
    target = raw.get("target") if isinstance(raw.get("target"), dict) else raw
    return {
        "x_mm": float(target.get("x_mm", target.get("x", 0.0))),
        "y_mm": float(target.get("y_mm", target.get("y", 0.0))),
        "z_mm": float(target.get("z_mm", target.get("z", 0.0))),
        "phi_deg": float(target.get("phi_deg", target.get("phi", 0.0))),
    }


def _safe_waypoint(config: RobotConfig) -> dict[str, Any]:
    settings = task_defaults(config)
    positions = named_positions(config)
    safe = positions.get(str(settings.get("safe_position", "safe")), {})
    if str(safe.get("type", "joint")).lower() == "joint":
        return {"type": "joint", "mode": "joint", "angles_deg": safe.get("angles_deg", config.home_pose)}
    return {"type": "cartesian", "mode": "joint", "target": _cartesian_target(safe)}


def build_pick_and_place_sequence(
    config: RobotConfig,
    object_target: dict[str, Any],
    drop_zone: str | dict[str, Any] | None = None,
) -> dict[str, Any]:
    settings = task_defaults(config)
    zones = drop_zones(config)
    object_pose = _cartesian_target(object_target)
    zone_name = str(drop_zone or settings.get("default_drop_zone", "dropoff_a"))
    if isinstance(drop_zone, dict):
        drop_pose = _cartesian_target(drop_zone)
        zone_name = "custom"
    elif zone_name in zones:
        drop_pose = zones[zone_name]
    else:
        return {"ok": False, "errors": [f"unknown drop zone {zone_name}"], "steps": [], "waypoints": []}

    approach_height = float(settings.get("approach_height_mm", 80.0))
    pickup_height = float(settings.get("pickup_height_mm", 25.0))
    dropoff_height = float(settings.get("dropoff_height_mm", drop_pose["z_mm"]))

    above_pick = {**object_pose, "z_mm": max(object_pose["z_mm"], approach_height)}
    at_pick = {**object_pose, "z_mm": pickup_height}
    above_drop = {**drop_pose, "z_mm": max(drop_pose["z_mm"], approach_height)}
    at_drop = {**drop_pose, "z_mm": dropoff_height}
    safe = _safe_waypoint(config)

    steps: list[dict[str, Any]] = [
        {"kind": "move", "label": "safe", "waypoint": safe},
        {"kind": "tool", "label": "open gripper", "action": "open"},
        {"kind": "move", "label": "above pickup", "waypoint": {"type": "cartesian", "mode": "joint", "target": above_pick}},
        {"kind": "move", "label": "pickup", "waypoint": {"type": "cartesian", "mode": "linear", "target": at_pick}},
        {"kind": "tool", "label": "close gripper", "action": "close"},
        {"kind": "move", "label": "lift", "waypoint": {"type": "cartesian", "mode": "linear", "target": above_pick}},
        {"kind": "move", "label": "above dropoff", "waypoint": {"type": "cartesian", "mode": "joint", "target": above_drop}},
        {"kind": "move", "label": "dropoff", "waypoint": {"type": "cartesian", "mode": "linear", "target": at_drop}},
        {"kind": "tool", "label": "open gripper", "action": "open"},
        {"kind": "move", "label": "lift from dropoff", "waypoint": {"type": "cartesian", "mode": "linear", "target": above_drop}},
        {"kind": "move", "label": "safe", "waypoint": safe},
    ]
    waypoints = [step["waypoint"] for step in steps if step["kind"] == "move"]
    return {
        "ok": True,
        "task": "pick_and_place",
        "drop_zone": zone_name,
        "steps": steps,
        "waypoints": waypoints,
        "object_target": object_pose,
    }


def build_sorting_sequence(
    config: RobotConfig,
    detection: dict[str, Any],
    color_profiles: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    color = str(detection.get("color", ""))
    if color not in color_profiles:
        return {"ok": False, "errors": [f"unknown color profile {color}"], "steps": [], "waypoints": []}
    robot = detection.get("robot") or detection.get("target") or detection
    profile = color_profiles[color]
    drop_zone = profile.get("drop_zone")
    sequence = build_pick_and_place_sequence(config, robot, drop_zone)
    sequence["task"] = "sorting"
    sequence["color"] = color
    return sequence
