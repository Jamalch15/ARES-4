from __future__ import annotations

from math import atan2, cos, degrees, hypot, isfinite, radians, sin
from typing import Any

import numpy as np

from .config import DHRowConfig, JointConfig, LinkConfig


COORDINATE_FRAME = """
Standard DH kinematics with the project robot frame mapped from the DH frame:
- The DH table uses theta, d, a, alpha in the standard convention.
- Joint values are UI/API degrees and are added to each row's theta_offset_deg.
- Robot coordinates are mapped as robot x = DH y, robot y = -DH x, robot z = DH z.
- At base/theta1 = 0 deg, positive planar reach points along global +Y.
- +Z points upward from the mounting plane.
- Working assumption for the measured L convention: d1=L1+L3, d2=s4*L4,
  a2=L5, d3=s6*L6, a3=L7, d4=s8*L8, a4=L9. L2 is recorded but is
  not yet part of the active DH table.
- tool_phi_deg is the first-pass pitch task angle theta2 + theta3 + theta4 after direction and zero offsets.
- The active tool TCP offset is applied after the final DH joint transform.
- Tool TCP +Z is treated as the tool-forward axis, which maps to local DH +X.
""".strip()


def _default_rows(links: LinkConfig) -> list[DHRowConfig]:
    return [
        DHRowConfig(0, 0.0, links.base_height_mm, 0.0, 90.0),
        DHRowConfig(1, 90.0, 0.0, links.upper_arm_mm, 0.0),
        DHRowConfig(2, 0.0, 0.0, links.forearm_mm, 0.0),
        DHRowConfig(3, 0.0, 0.0, links.wrist_mm + links.tool_mm, 0.0),
    ]


def _rows(links: LinkConfig) -> list[DHRowConfig]:
    return links.dh_rows if links.dh_rows else _default_rows(links)


def _normalize_deg(angle: float) -> float:
    normalized = (angle + 180.0) % 360.0 - 180.0
    if normalized == -180.0 and angle > 0:
        return 180.0
    return normalized


def angle_distance_deg(a: float, b: float) -> float:
    return abs(_normalize_deg(a - b))


def _signed_angle_error_deg(target: float, actual: float) -> float:
    return _normalize_deg(target - actual)


def _dh_matrix(theta_deg: float, d_mm: float, a_mm: float, alpha_deg: float) -> np.ndarray:
    return (
        _rotation_z(theta_deg)
        @ _translation(0.0, 0.0, d_mm)
        @ _translation(a_mm, 0.0, 0.0)
        @ _rotation_x(alpha_deg)
    )


def _rotation_z(theta_deg: float) -> np.ndarray:
    theta = radians(theta_deg)
    ct = cos(theta)
    st = sin(theta)
    return np.array(
        [
            [ct, -st, 0.0, 0.0],
            [st, ct, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _rotation_x(alpha_deg: float) -> np.ndarray:
    alpha = radians(alpha_deg)
    ca = cos(alpha)
    sa = sin(alpha)
    return np.array(
        [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, ca, -sa, 0.0],
            [0.0, sa, ca, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        dtype=float,
    )


def _translation(x_mm: float, y_mm: float, z_mm: float) -> np.ndarray:
    transform = np.identity(4)
    transform[0, 3] = x_mm
    transform[1, 3] = y_mm
    transform[2, 3] = z_mm
    return transform


def _dh_step(
    transform: np.ndarray,
    theta_deg: float,
    d_mm: float,
    a_mm: float,
    alpha_deg: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    after_theta = transform @ _rotation_z(theta_deg)
    after_d = after_theta @ _translation(0.0, 0.0, d_mm)
    after_a = after_d @ _translation(a_mm, 0.0, 0.0)
    final = after_a @ _rotation_x(alpha_deg)
    return final, after_d, after_a


def _robot_point_from_dh(vector: np.ndarray) -> dict[str, float]:
    return {"x_mm": float(vector[1]), "y_mm": float(-vector[0]), "z_mm": float(vector[2])}


def _row_theta(row: DHRowConfig, joint_angles_deg: list[float]) -> float:
    joint_angle = float(joint_angles_deg[row.joint_index])
    return joint_angle * row.direction_sign + row.zero_offset_deg + row.theta_offset_deg


def dh_transforms(joint_angles_deg: list[float], links: LinkConfig) -> list[np.ndarray]:
    if len(joint_angles_deg) != 4:
        raise ValueError("DH kinematics expects four joint angles")
    transforms: list[np.ndarray] = []
    transform = np.identity(4)
    transforms.append(transform.copy())
    for row in _rows(links):
        transform, _, _ = _dh_step(
            transform,
            _row_theta(row, joint_angles_deg),
            row.d_mm,
            row.a_mm,
            row.alpha_deg,
        )
        transforms.append(transform.copy())
    return transforms


def joint_frame_points(joint_angles_deg: list[float], links: LinkConfig) -> list[dict[str, float]]:
    return [_robot_point_from_dh(transform[:3, 3]) for transform in dh_transforms(joint_angles_deg, links)]


def _segment_label(row_index: int, kind: str) -> str:
    labels = [
        {"d": "L1+L3", "a": "a1"},
        {"d": "s4*L4", "a": "L5"},
        {"d": "s6*L6", "a": "L7"},
        {"d": "s8*L8", "a": "L9"},
    ]
    if row_index < len(labels):
        return labels[row_index].get(kind, f"{kind}{row_index + 1}")
    return f"{kind}{row_index + 1}"


def dh_segment_points(joint_angles_deg: list[float], links: LinkConfig) -> list[dict[str, Any]]:
    """Return Standard DH translation segments as separate d and a offsets."""
    if len(joint_angles_deg) != 4:
        raise ValueError("DH segment visualization expects four joint angles")
    transform = np.identity(4)
    segments: list[dict[str, Any]] = []
    for row_index, row in enumerate(_rows(links)):
        final, after_d, after_a = _dh_step(
            transform,
            _row_theta(row, joint_angles_deg),
            row.d_mm,
            row.a_mm,
            row.alpha_deg,
        )
        start = _robot_point_from_dh(transform[:3, 3])
        d_end = _robot_point_from_dh(after_d[:3, 3])
        a_end = _robot_point_from_dh(after_a[:3, 3])
        if abs(row.d_mm) > 1e-9:
            segments.append(
                {
                    "kind": "d",
                    "row": row_index + 1,
                    "joint": row.joint_index + 1,
                    "label": _segment_label(row_index, "d"),
                    "signed_length_mm": float(row.d_mm),
                    "length_mm": abs(float(row.d_mm)),
                    "start": start,
                    "end": d_end,
                }
            )
        if abs(row.a_mm) > 1e-9:
            segments.append(
                {
                    "kind": "a",
                    "row": row_index + 1,
                    "joint": row.joint_index + 1,
                    "label": _segment_label(row_index, "a"),
                    "signed_length_mm": float(row.a_mm),
                    "length_mm": abs(float(row.a_mm)),
                    "start": d_end,
                    "end": a_end,
                }
            )
        transform = final
    return segments


def _tool_phi_from_angles(joint_angles_deg: list[float], links: LinkConfig) -> float:
    rows = _rows(links)
    pitch = 0.0
    for row in rows:
        if row.joint_index > 0:
            pitch += float(joint_angles_deg[row.joint_index]) * row.direction_sign + row.zero_offset_deg
    return _normalize_deg(pitch)


def _tool_tcp_offset_vector(links: LinkConfig) -> np.ndarray:
    offset = links.tool_tcp_offset_mm or {}
    tool_x = float(offset.get("x", offset.get("x_mm", 0.0)))
    tool_y = float(offset.get("y", offset.get("y_mm", 0.0)))
    tool_z = float(offset.get("z", offset.get("z_mm", 0.0)))
    # The UI/config uses tool +Z as the forward TCP axis. Standard DH link
    # extension uses local +X, so map tool-frame offsets into the final DH frame.
    return np.array(
        [
            tool_z,
            tool_x,
            tool_y,
            1.0,
        ],
        dtype=float,
    )


def forward_kinematics(joint_angles_deg: list[float], links: LinkConfig) -> dict[str, Any]:
    frames = dh_transforms(joint_angles_deg, links)
    wrist = _robot_point_from_dh(frames[-1][:3, 3])
    tcp_vector = frames[-1] @ _tool_tcp_offset_vector(links)
    tcp = _robot_point_from_dh(tcp_vector[:3])
    phi_deg = _tool_phi_from_angles(joint_angles_deg, links)
    tcp["radial_mm"] = hypot(tcp["x_mm"], tcp["y_mm"])
    tcp["tool_phi_deg"] = phi_deg
    tcp["tool_pitch_deg"] = phi_deg
    tcp["dh_frames"] = joint_frame_points(joint_angles_deg, links)
    tcp["dh_segments"] = dh_segment_points(joint_angles_deg, links)
    tcp["wrist_frame"] = wrist
    tcp["tool_tcp_offset_mm"] = dict(links.tool_tcp_offset_mm or {})
    return tcp


def _joint_limit_reasons(joints: list[JointConfig], angles_deg: list[float]) -> list[str]:
    reasons: list[str] = []
    for joint, angle in zip(joints, angles_deg, strict=True):
        if angle < joint.min_deg or angle > joint.max_deg:
            reasons.append(
                f"{joint.name} {angle:.2f} deg outside {joint.min_deg:.2f}..{joint.max_deg:.2f} deg"
            )
    return reasons


def _task_error(target: dict[str, float], angles_deg: list[float], links: LinkConfig) -> np.ndarray:
    fk = forward_kinematics(angles_deg, links)
    return np.array(
        [
            float(target["x_mm"]) - fk["x_mm"],
            float(target["y_mm"]) - fk["y_mm"],
            float(target["z_mm"]) - fk["z_mm"],
            _signed_angle_error_deg(float(target["phi_deg"]), fk["tool_phi_deg"]),
        ],
        dtype=float,
    )


def _numeric_jacobian(target: dict[str, float], angles_deg: list[float], links: LinkConfig) -> np.ndarray:
    del target
    base_fk = forward_kinematics(angles_deg, links)
    base_vector = np.array(
        [base_fk["x_mm"], base_fk["y_mm"], base_fk["z_mm"], base_fk["tool_phi_deg"]],
        dtype=float,
    )
    jacobian = np.zeros((4, len(angles_deg)), dtype=float)
    eps = 0.05
    for index in range(len(angles_deg)):
        shifted = angles_deg.copy()
        shifted[index] += eps
        shifted_fk = forward_kinematics(shifted, links)
        shifted_vector = np.array(
            [
                shifted_fk["x_mm"],
                shifted_fk["y_mm"],
                shifted_fk["z_mm"],
                shifted_fk["tool_phi_deg"],
            ],
            dtype=float,
        )
        diff = shifted_vector - base_vector
        diff[3] = _normalize_deg(shifted_fk["tool_phi_deg"] - base_fk["tool_phi_deg"])
        jacobian[:, index] = diff / eps
    return jacobian


def _clamp_to_limits(angles_deg: list[float], joints: list[JointConfig]) -> list[float]:
    return [max(joint.min_deg, min(joint.max_deg, angle)) for angle, joint in zip(angles_deg, joints, strict=True)]


def _solve_from_seed(
    target: dict[str, float],
    links: LinkConfig,
    joints: list[JointConfig],
    seed_deg: list[float],
    label: str,
    max_iterations: int = 160,
    damping: float = 0.18,
    position_tolerance_mm: float = 1.0,
    orientation_tolerance_deg: float = 1.0,
) -> dict[str, Any]:
    angles = _clamp_to_limits([float(value) for value in seed_deg], joints)
    notes: list[str] = []
    iterations = 0
    singular = False

    for iterations in range(1, max_iterations + 1):
        error = _task_error(target, angles, links)
        position_error = float(np.linalg.norm(error[:3]))
        orientation_error = abs(float(error[3]))
        if position_error <= position_tolerance_mm and orientation_error <= orientation_tolerance_deg:
            break

        jacobian = _numeric_jacobian(target, angles, links)
        condition = np.linalg.cond(jacobian @ jacobian.T + np.identity(4) * 1e-9)
        singular = singular or bool(condition > 1e8)
        lhs = jacobian @ jacobian.T + (damping**2) * np.identity(4)
        try:
            delta = jacobian.T @ np.linalg.solve(lhs, error)
        except np.linalg.LinAlgError:
            notes.append("jacobian solve failed")
            break
        delta = np.clip(delta, -8.0, 8.0)
        next_angles = _clamp_to_limits([angle + float(step) for angle, step in zip(angles, delta, strict=True)], joints)
        if all(abs(a - b) < 1e-6 for a, b in zip(next_angles, angles, strict=True)):
            notes.append("solver stalled at a joint limit")
            break
        angles = next_angles

    fk = forward_kinematics(angles, links)
    position_error = hypot(hypot(fk["x_mm"] - target["x_mm"], fk["y_mm"] - target["y_mm"]), fk["z_mm"] - target["z_mm"])
    phi_error = angle_distance_deg(fk["tool_phi_deg"], target["phi_deg"])
    reasons = _joint_limit_reasons(joints, angles)
    if position_error > position_tolerance_mm or phi_error > orientation_tolerance_deg:
        reasons.append(
            f"IK did not converge: position error {position_error:.2f} mm, phi error {phi_error:.2f} deg"
        )
    if singular:
        notes.append("near-singular Jacobian encountered")
    return {
        "branch": label,
        "angles_deg": angles,
        "valid": not reasons,
        "reasons": reasons,
        "fk": fk,
        "position_error_mm": position_error,
        "phi_error_deg": phi_error,
        "iterations": iterations,
        "singularity_warning": singular,
        "notes": notes,
    }


def _seed_candidates(target: dict[str, float], joints: list[JointConfig], current: list[float]) -> list[tuple[str, list[float]]]:
    base_guess = degrees(atan2(-target["x_mm"], target["y_mm"])) if hypot(target["x_mm"], target["y_mm"]) > 1e-6 else current[0]
    home = [joint.home_deg for joint in joints]
    return [
        ("current_seed", current),
        ("elbow_up", [base_guess, min(60.0, joints[1].max_deg), 35.0, -20.0]),
        ("elbow_down", [base_guess, 30.0, -55.0, 55.0]),
        ("home_seed", home),
    ]


def inverse_kinematics(
    target: dict[str, float],
    links: LinkConfig,
    joints: list[JointConfig],
    current_joints_deg: list[float] | None = None,
    branch: str = "auto",
    tolerance: float = 1e-6,
) -> dict[str, Any]:
    del tolerance
    if len(joints) != 4:
        raise ValueError("inverse_kinematics expects four joint configs")

    pose = {
        "x_mm": float(target.get("x_mm", 0.0)),
        "y_mm": float(target.get("y_mm", 0.0)),
        "z_mm": float(target.get("z_mm", 0.0)),
        "phi_deg": float(target.get("phi_deg", target.get("tool_phi_deg", 0.0))),
    }
    if not all(isfinite(value) for value in pose.values()):
        return {
            "ok": False,
            "target": pose,
            "candidates": [],
            "selected": None,
            "selected_branch": None,
            "notes": ["target contains a non-finite value"],
        }

    current = current_joints_deg or [joint.home_deg for joint in joints]
    seeds = _seed_candidates(pose, joints, [float(value) for value in current])
    requested_branch = branch if branch in {"elbow_up", "elbow_down", "current_seed", "home_seed"} else "auto"
    candidates = [
        _solve_from_seed(pose, links, joints, seed, label)
        for label, seed in seeds
    ]
    valid_candidates = [candidate for candidate in candidates if candidate["valid"]]
    selected: dict[str, Any] | None = None
    notes: list[str] = []

    if requested_branch != "auto":
        selected = next((candidate for candidate in valid_candidates if candidate["branch"] == requested_branch), None)
        if selected is None:
            notes.append(f"requested branch {requested_branch} is not valid")
    if selected is None and valid_candidates:
        selected = min(
            valid_candidates,
            key=lambda candidate: (
                candidate["position_error_mm"],
                candidate["phi_error_deg"],
                sum(angle_distance_deg(angle, current[index]) for index, angle in enumerate(candidate["angles_deg"])),
            ),
        )

    if selected is None:
        notes.insert(0, "target is unreachable or did not converge in DH/Jacobian solver")

    return {
        "ok": selected is not None,
        "target": pose,
        "candidates": candidates,
        "selected": selected,
        "selected_branch": selected["branch"] if selected else None,
        "notes": notes,
    }
