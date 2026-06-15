from __future__ import annotations

from math import atan2, cos, degrees, hypot, isfinite, pi, radians, sin, sqrt
from typing import Any

from .config import JointConfig, LinkConfig


COORDINATE_FRAME = """
Working assumption for the simple IK/FK model:
- Origin is the center of the base rotation axis on the robot mounting plane.
- +Z points upward.
- At base/theta1 = 0 deg, the arm's planar working direction points along global +Y.
- +X is horizontal sideways after base rotation.
- theta2 is measured from vertical +Z toward the planar horizontal reach direction.
- theta3 and theta4 are relative pitch angles in the same side plane.
- phi is the tool angle measured from the local horizontal reach axis.
- Internally, trigonometry uses radians; UI/API values use millimeters and degrees.
This is the first simplified planar + base-rotation model, not a finalized calibration.
""".strip()


def _tool_length_mm(links: LinkConfig) -> float:
    return links.wrist_mm + links.tool_mm


def _normalize_deg(angle: float) -> float:
    normalized = (angle + 180.0) % 360.0 - 180.0
    if normalized == -180.0 and angle > 0:
        return 180.0
    return normalized


def angle_distance_deg(a: float, b: float) -> float:
    return abs(_normalize_deg(a - b))


def _joint_limit_reasons(joints: list[JointConfig], angles_deg: list[float]) -> list[str]:
    reasons: list[str] = []
    for joint, angle in zip(joints, angles_deg, strict=True):
        if angle < joint.min_deg or angle > joint.max_deg:
            reasons.append(
                f"{joint.name} {angle:.2f} deg outside {joint.min_deg:.2f}..{joint.max_deg:.2f} deg"
            )
    return reasons


def forward_kinematics(joint_angles_deg: list[float], links: LinkConfig) -> dict[str, Any]:
    if len(joint_angles_deg) != 4:
        raise ValueError("forward_kinematics expects four joint angles")

    base, shoulder, elbow, wrist = [radians(angle) for angle in joint_angles_deg]
    wrist_total_mm = _tool_length_mm(links)

    shoulder_pitch = shoulder
    elbow_pitch = shoulder + elbow
    wrist_pitch = shoulder + elbow + wrist

    radial_mm = (
        links.upper_arm_mm * sin(shoulder_pitch)
        + links.forearm_mm * sin(elbow_pitch)
        + wrist_total_mm * sin(wrist_pitch)
    )
    z_mm = (
        links.base_height_mm
        + links.upper_arm_mm * cos(shoulder_pitch)
        + links.forearm_mm * cos(elbow_pitch)
        + wrist_total_mm * cos(wrist_pitch)
    )
    x_mm = -radial_mm * sin(base)
    y_mm = radial_mm * cos(base)
    phi_deg = _normalize_deg(90.0 - sum(joint_angles_deg[1:]))

    return {
        "x_mm": x_mm,
        "y_mm": y_mm,
        "z_mm": z_mm,
        "radial_mm": radial_mm,
        "tool_phi_deg": phi_deg,
        "tool_pitch_deg": phi_deg,
    }


def inverse_kinematics(
    target: dict[str, float],
    links: LinkConfig,
    joints: list[JointConfig],
    current_joints_deg: list[float] | None = None,
    branch: str = "auto",
    tolerance: float = 1e-6,
) -> dict[str, Any]:
    if len(joints) != 4:
        raise ValueError("inverse_kinematics expects four joint configs")

    x_mm = float(target.get("x_mm", 0.0))
    y_mm = float(target.get("y_mm", 0.0))
    z_mm = float(target.get("z_mm", 0.0))
    phi_deg = float(target.get("phi_deg", target.get("tool_phi_deg", 0.0)))
    phi = radians(phi_deg)
    current = current_joints_deg or [joint.home_deg for joint in joints]
    notes: list[str] = []

    if not all(isfinite(value) for value in [x_mm, y_mm, z_mm, phi_deg]):
        return {
            "ok": False,
            "target": {"x_mm": x_mm, "y_mm": y_mm, "z_mm": z_mm, "phi_deg": phi_deg},
            "candidates": [],
            "selected": None,
            "selected_branch": None,
            "notes": ["target contains a non-finite value"],
        }

    l2 = links.upper_arm_mm
    l3 = links.forearm_mm
    l4 = _tool_length_mm(links)
    if l2 <= 0 or l3 <= 0:
        return {
            "ok": False,
            "target": {"x_mm": x_mm, "y_mm": y_mm, "z_mm": z_mm, "phi_deg": phi_deg},
            "candidates": [],
            "selected": None,
            "selected_branch": None,
            "notes": ["upper_arm and forearm lengths must be positive"],
        }

    radial_mm = hypot(x_mm, y_mm)
    if radial_mm < tolerance:
        theta1 = radians(current[0])
        notes.append("base angle is ambiguous at r=0; using current base angle")
    else:
        theta1 = atan2(-x_mm, y_mm)

    h_mm = z_mm - links.base_height_mm
    wrist_r_mm = radial_mm - l4 * cos(phi)
    wrist_h_mm = h_mm - l4 * sin(phi)
    d = (wrist_r_mm**2 + wrist_h_mm**2 - l2**2 - l3**2) / (2.0 * l2 * l3)

    if d > 1.0 + tolerance or d < -1.0 - tolerance:
        return {
            "ok": False,
            "target": {"x_mm": x_mm, "y_mm": y_mm, "z_mm": z_mm, "phi_deg": phi_deg},
            "wrist_target": {"r_mm": wrist_r_mm, "h_mm": wrist_h_mm, "d": d},
            "candidates": [],
            "selected": None,
            "selected_branch": None,
            "notes": notes + [f"target is unreachable in simplified model, D={d:.4f}"],
        }

    d = max(-1.0, min(1.0, d))
    root = sqrt(max(0.0, 1.0 - d**2))
    branch_specs = [("elbow_up", root), ("elbow_down", -root)]
    candidates: list[dict[str, Any]] = []

    for branch_name, root_sign in branch_specs:
        theta3 = atan2(root_sign, d)
        theta2 = atan2(wrist_r_mm, wrist_h_mm) - atan2(l3 * sin(theta3), l2 + l3 * cos(theta3))
        theta4 = (pi / 2.0) - phi - theta2 - theta3
        angles = [
            _normalize_deg(degrees(theta1)),
            degrees(theta2),
            degrees(theta3),
            degrees(theta4),
        ]
        reasons = _joint_limit_reasons(joints, angles)
        fk = forward_kinematics(angles, links)
        position_error = hypot(
            hypot(fk["x_mm"] - x_mm, fk["y_mm"] - y_mm),
            fk["z_mm"] - z_mm,
        )
        phi_error = angle_distance_deg(fk["tool_phi_deg"], phi_deg)
        distance = sum(angle_distance_deg(angle, current[index]) for index, angle in enumerate(angles))
        candidates.append(
            {
                "branch": branch_name,
                "angles_deg": angles,
                "valid": not reasons,
                "reasons": reasons,
                "fk": fk,
                "position_error_mm": position_error,
                "phi_error_deg": phi_error,
                "distance_from_current_deg": distance,
            }
        )

    valid_candidates = [candidate for candidate in candidates if candidate["valid"]]
    selected: dict[str, Any] | None = None
    requested_branch = branch if branch in {"elbow_up", "elbow_down"} else "auto"
    if requested_branch != "auto":
        selected = next(
            (candidate for candidate in valid_candidates if candidate["branch"] == requested_branch),
            None,
        )
        if selected is None:
            notes.append(f"requested branch {requested_branch} is not valid")
    if selected is None and valid_candidates:
        selected = min(valid_candidates, key=lambda candidate: candidate["distance_from_current_deg"])

    return {
        "ok": selected is not None,
        "target": {"x_mm": x_mm, "y_mm": y_mm, "z_mm": z_mm, "phi_deg": phi_deg},
        "wrist_target": {"r_mm": wrist_r_mm, "h_mm": wrist_h_mm, "d": d},
        "candidates": candidates,
        "selected": selected,
        "selected_branch": selected["branch"] if selected else None,
        "notes": notes if notes else ([] if selected else ["no valid IK branch after joint-limit filtering"]),
    }
