from __future__ import annotations

from typing import Any

import numpy as np

from .cartesian_servo import CartesianServo, CartesianServoLimits
from .config import RobotConfig
from .kinematics import forward_kinematics


def _fk_position(joints_deg: list[float], config: RobotConfig) -> list[float]:
    fk = forward_kinematics(joints_deg, config.links)
    return [float(fk["x_mm"]), float(fk["y_mm"]), float(fk["z_mm"])]


def _fk_pose(joints_deg: list[float], config: RobotConfig) -> dict[str, float]:
    fk = forward_kinematics(joints_deg, config.links)
    return {
        "x_mm": float(fk["x_mm"]),
        "y_mm": float(fk["y_mm"]),
        "z_mm": float(fk["z_mm"]),
        "phi_deg": float(fk["tool_phi_deg"]),
    }


def cartesian_path_metrics(points: list[list[float]], direction_xyz: list[float]) -> dict[str, float | int]:
    """Measure how well a sampled TCP path follows the requested Cartesian direction."""
    if len(points) < 2:
        return {
            "progress_mm": 0.0,
            "lateral_mm": 0.0,
            "max_lateral_mm": 0.0,
            "alignment": 0.0,
            "backward_steps": 0,
        }

    direction = np.array([float(value) for value in direction_xyz], dtype=float)
    direction_norm = float(np.linalg.norm(direction))
    if direction_norm <= 1e-9:
        raise ValueError("direction_xyz must contain a non-zero xyz direction")
    unit = direction / direction_norm
    start = np.array(points[0], dtype=float)
    end = np.array(points[-1], dtype=float)
    displacement = end - start
    progress = float(displacement @ unit)
    lateral = float(np.linalg.norm(displacement - progress * unit))
    displacement_norm = float(np.linalg.norm(displacement))
    alignment = progress / max(displacement_norm, 1e-9)

    max_lateral = 0.0
    backward_steps = 0
    previous_progress = 0.0
    for point in points[1:]:
        relative = np.array(point, dtype=float) - start
        point_progress = float(relative @ unit)
        point_lateral = float(np.linalg.norm(relative - point_progress * unit))
        max_lateral = max(max_lateral, point_lateral)
        if point_progress < previous_progress - 1e-3:
            backward_steps += 1
        previous_progress = point_progress

    return {
        "progress_mm": progress,
        "lateral_mm": lateral,
        "max_lateral_mm": max_lateral,
        "alignment": alignment,
        "backward_steps": backward_steps,
    }


def simulate_cartesian_jog(
    config: RobotConfig,
    start_deg: list[float],
    velocity_xyz_mm_s: list[float],
    *,
    vphi_deg_s: float = 0.0,
    steps: int = 36,
    dt_s: float = 1.0 / 12.0,
    damping: float | None = None,
    apply_joint_limits: bool = True,
) -> dict[str, Any]:
    """Run the production Cartesian servo deterministically without FastAPI.

    ``damping`` remains accepted for compatibility with older scripts but is no
    longer used: the replacement solver preserves task direction exactly and
    scales speed against explicit bounds instead of damping into lateral drift.
    """
    del damping
    if len(start_deg) != len(config.joints):
        raise ValueError(f"expected {len(config.joints)} start joint angles")
    if len(velocity_xyz_mm_s) != 3:
        raise ValueError("velocity_xyz_mm_s must contain x/y/z velocities")
    if steps < 1:
        raise ValueError("steps must be positive")
    if dt_s <= 0.0:
        raise ValueError("dt_s must be positive")

    servo = CartesianServo(config.links, config.joints, [float(value) for value in start_deg])
    servo.set_command(
        [
            float(velocity_xyz_mm_s[0]),
            float(velocity_xyz_mm_s[1]),
            float(velocity_xyz_mm_s[2]),
            float(vphi_deg_s),
        ]
    )
    speed_limits = [
        float(joint.max_speed_deg_s) if apply_joint_limits else 1e6
        for joint in config.joints
    ]
    limits = CartesianServoLimits(
        joint_speed_deg_s=speed_limits,
        tcp_accel_mm_s2=360.0,
        phi_accel_deg_s2=240.0,
    )
    current = servo.commanded_joints_deg.tolist()
    points = [_fk_position(current, config)]
    samples: list[dict[str, Any]] = []
    notes: list[str] = []
    blocked_steps = 0

    for step_index in range(steps):
        servo_step = servo.step(dt_s, limits)
        current = [float(value) for value in servo_step["target_angles_deg"]]
        step_notes = [str(note) for note in servo_step.get("notes", [])]
        if servo_step.get("failure_reason"):
            step_notes.append(str(servo_step["failure_reason"]))
        if servo_step.get("blocked"):
            blocked_steps += 1
        notes.extend(str(note) for note in step_notes)
        points.append(_fk_position(current, config))
        samples.append(
            {
                "step": step_index,
                "joints_deg": current,
                "joint_velocity_deg_s": servo_step["joint_velocity_deg_s"],
                "servo": servo_step,
                "notes": step_notes,
            }
        )

    metrics = cartesian_path_metrics(points, velocity_xyz_mm_s)
    return {
        "start_deg": [float(value) for value in start_deg],
        "final_deg": current,
        "velocity_xyz_mm_s": [float(value) for value in velocity_xyz_mm_s],
        "vphi_deg_s": float(vphi_deg_s),
        "dt_s": float(dt_s),
        "steps": len(samples),
        "points": points,
        "samples": samples,
        "blocked_steps": blocked_steps,
        "notes": list(dict.fromkeys(notes)),
        "metrics": metrics,
    }
