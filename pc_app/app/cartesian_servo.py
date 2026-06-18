from __future__ import annotations

from dataclasses import dataclass
from itertools import combinations, product
from math import isfinite
from typing import Any

import numpy as np

from .config import JointConfig, LinkConfig
from .kinematics import forward_kinematics, geometric_task_jacobian, inverse_kinematics


_SOLUTION_TOLERANCE = 1e-7
_DIRECTION_TOLERANCE = 0.995
_MIN_USEFUL_SCALE = 1e-4


def _ramp_vector(current: np.ndarray, target: np.ndarray, max_delta: float) -> np.ndarray:
    delta = target - current
    magnitude = float(np.linalg.norm(delta))
    if magnitude <= max_delta or magnitude <= 1e-12:
        return target.copy()
    return current + delta * (max_delta / magnitude)


def _ramp_scalar(current: float, target: float, max_delta: float) -> float:
    delta = target - current
    if abs(delta) <= max_delta:
        return target
    return current + (max_delta if delta > 0.0 else -max_delta)


def _closest_bounded_equality_solution(
    matrix: np.ndarray,
    target: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    reference: np.ndarray,
) -> np.ndarray | None:
    """Solve a tiny bounded equality problem by enumerating active bounds.

    The robot has four joints, so checking all 3^4 free/lower/upper active-set
    combinations is deterministic, fast, and avoids adding a heavyweight QP
    dependency. Among exact feasible solutions it chooses the one nearest the
    supplied reference.
    """
    variable_count = matrix.shape[1]
    _, singular_values, right_vectors = np.linalg.svd(matrix, full_matrices=True)
    rank = int(np.sum(singular_values > 1e-10))
    particular = np.linalg.pinv(matrix, rcond=1e-10) @ target
    target_scale = max(1.0, float(np.linalg.norm(target)))
    if float(np.linalg.norm(matrix @ particular - target)) > _SOLUTION_TOLERANCE * target_scale:
        return None

    nullity = variable_count - rank
    if nullity == 0:
        if np.any(particular < lower - 1e-8) or np.any(particular > upper + 1e-8):
            return None
        return np.minimum(upper, np.maximum(lower, particular))
    if nullity == 1:
        null_vector = right_vectors[rank, :]
        interval_low = float("-inf")
        interval_high = float("inf")
        for index, coefficient in enumerate(null_vector):
            if abs(float(coefficient)) <= 1e-12:
                if particular[index] < lower[index] - 1e-8 or particular[index] > upper[index] + 1e-8:
                    return None
                continue
            first = (lower[index] - particular[index]) / coefficient
            second = (upper[index] - particular[index]) / coefficient
            interval_low = max(interval_low, min(first, second))
            interval_high = min(interval_high, max(first, second))
        if interval_low > interval_high + 1e-10:
            return None
        preferred = float(null_vector @ (reference - particular))
        parameter = min(interval_high, max(interval_low, preferred))
        candidate = particular + null_vector * parameter
        return np.minimum(upper, np.maximum(lower, candidate))

    best: np.ndarray | None = None
    best_cost = float("inf")

    for statuses in product((-1, 0, 1), repeat=variable_count):
        fixed = [index for index, status in enumerate(statuses) if status != 0]
        free = [index for index, status in enumerate(statuses) if status == 0]
        candidate = np.zeros(variable_count, dtype=float)

        for index in fixed:
            candidate[index] = lower[index] if statuses[index] < 0 else upper[index]

        rhs = target.copy()
        if fixed:
            rhs = rhs - matrix[:, fixed] @ candidate[fixed]

        if free:
            free_matrix = matrix[:, free]
            free_reference = reference[free]
            residual = rhs - free_matrix @ free_reference
            correction = np.linalg.pinv(free_matrix, rcond=1e-10) @ residual
            candidate[free] = free_reference + correction

        equality_error = float(np.linalg.norm(matrix @ candidate - target))
        if equality_error > _SOLUTION_TOLERANCE * target_scale:
            continue
        if np.any(candidate < lower - 1e-8) or np.any(candidate > upper + 1e-8):
            continue

        candidate = np.minimum(upper, np.maximum(lower, candidate))
        cost = float(np.dot(candidate - reference, candidate - reference))
        if cost < best_cost:
            best = candidate
            best_cost = cost
    return best


def solve_direction_preserving_velocity(
    jacobian: np.ndarray,
    requested_task_velocity: np.ndarray,
    lower_joint_velocity: np.ndarray,
    upper_joint_velocity: np.ndarray,
    reference_joint_velocity: np.ndarray | None = None,
) -> tuple[np.ndarray, float]:
    """Find the fastest feasible joint velocity with exact task direction.

    The equality is ``J qdot = scale * requested``. A binary search maximizes
    ``scale`` in ``[0, 1]`` while joint velocity and near-limit bounds remain
    satisfied. This prevents the lateral Cartesian drift produced by an
    unconstrained damped least-squares approximation.
    """
    requested = np.asarray(requested_task_velocity, dtype=float)
    lower = np.asarray(lower_joint_velocity, dtype=float)
    upper = np.asarray(upper_joint_velocity, dtype=float)
    reference = (
        np.zeros(jacobian.shape[1], dtype=float)
        if reference_joint_velocity is None
        else np.asarray(reference_joint_velocity, dtype=float)
    )
    if float(np.linalg.norm(requested)) <= 1e-12:
        return np.zeros(jacobian.shape[1], dtype=float), 1.0

    def feasible(scale: float) -> np.ndarray | None:
        return _closest_bounded_equality_solution(
            jacobian,
            requested * scale,
            lower,
            upper,
            reference * scale,
        )

    full = feasible(1.0)
    if full is not None:
        return full, 1.0

    # Maximize scale as a five-variable linear feasibility problem:
    # [J, -requested] [qdot, scale] = 0, with box bounds. A linear optimum
    # exists at a vertex, so only the nullity number of active box bounds must
    # be enumerated. For this 4DOF arm that is normally 1 or 2 bounds.
    extended_matrix = np.column_stack((jacobian, -requested))
    extended_lower = np.concatenate((lower, np.array([0.0])))
    extended_upper = np.concatenate((upper, np.array([1.0])))
    extended_reference = np.concatenate((reference, np.array([1.0])))
    rank = int(np.linalg.matrix_rank(extended_matrix, tol=1e-10))
    nullity = max(1, extended_matrix.shape[1] - rank)
    best_scale = 0.0

    for active_indices in combinations(range(extended_matrix.shape[1]), nullity):
        free_indices = [index for index in range(extended_matrix.shape[1]) if index not in active_indices]
        for bound_sides in product((-1, 1), repeat=nullity):
            candidate = np.zeros(extended_matrix.shape[1], dtype=float)
            for index, side in zip(active_indices, bound_sides, strict=True):
                candidate[index] = extended_lower[index] if side < 0 else extended_upper[index]

            rhs = -extended_matrix[:, active_indices] @ candidate[list(active_indices)]
            if free_indices:
                free_matrix = extended_matrix[:, free_indices]
                free_reference = extended_reference[free_indices]
                correction = np.linalg.pinv(free_matrix, rcond=1e-10) @ (
                    rhs - free_matrix @ free_reference
                )
                candidate[free_indices] = free_reference + correction

            if float(np.linalg.norm(extended_matrix @ candidate)) > _SOLUTION_TOLERANCE:
                continue
            if np.any(candidate < extended_lower - 1e-8) or np.any(candidate > extended_upper + 1e-8):
                continue
            best_scale = max(best_scale, float(candidate[-1]))

    if best_scale <= _MIN_USEFUL_SCALE:
        return np.zeros(jacobian.shape[1], dtype=float), 0.0
    best = feasible(min(1.0, best_scale))
    if best is None:
        # Numerical roundoff at the exact vertex can put the refined solve a
        # few ulps outside a bound.
        best_scale *= 1.0 - 1e-9
        best = feasible(best_scale)
    if best is None:
        return np.zeros(jacobian.shape[1], dtype=float), 0.0
    return best, best_scale


def _task_delta(start_fk: dict[str, Any], end_fk: dict[str, Any]) -> np.ndarray:
    phi_delta = (float(end_fk["tool_phi_deg"]) - float(start_fk["tool_phi_deg"]) + 180.0) % 360.0 - 180.0
    return np.array(
        [
            float(end_fk["x_mm"]) - float(start_fk["x_mm"]),
            float(end_fk["y_mm"]) - float(start_fk["y_mm"]),
            float(end_fk["z_mm"]) - float(start_fk["z_mm"]),
            phi_delta,
        ],
        dtype=float,
    )


@dataclass
class CartesianServoLimits:
    joint_speed_deg_s: list[float]
    tcp_accel_mm_s2: float = 360.0
    phi_accel_deg_s2: float = 240.0
    joint_limit_margin_deg: float = 0.25


class CartesianServo:
    """Stateful fixed-period Cartesian velocity controller.

    One commanded joint pose is authoritative. Cartesian commands are ramped
    before inverse kinematics, solved with exact direction constraints, and
    integrated once into the next synchronized joint position target.
    """

    def __init__(self, links: LinkConfig, joints: list[JointConfig], initial_joints_deg: list[float]):
        self.links = links
        self.joints = joints
        self.commanded_joints_deg = np.asarray(initial_joints_deg, dtype=float)
        self.target_task_velocity = np.zeros(4, dtype=float)
        self.applied_task_velocity = np.zeros(4, dtype=float)
        self.joint_velocity_deg_s = np.zeros(len(joints), dtype=float)
        self.last_result: dict[str, Any] = {}

    def reconfigure(self, links: LinkConfig, joints: list[JointConfig]) -> None:
        self.links = links
        self.joints = joints
        self.reset(self.commanded_joints_deg.tolist())

    def reset(self, joints_deg: list[float]) -> None:
        self.commanded_joints_deg = np.asarray(joints_deg, dtype=float)
        self.target_task_velocity = np.zeros(4, dtype=float)
        self.applied_task_velocity = np.zeros(4, dtype=float)
        self.joint_velocity_deg_s = np.zeros(len(self.joints), dtype=float)
        self.last_result = {}

    def set_command(self, velocity: list[float] | tuple[float, float, float, float]) -> None:
        if len(velocity) != 4 or not all(isfinite(float(value)) for value in velocity):
            raise ValueError("Cartesian servo velocity must contain four finite values")
        self.target_task_velocity = np.asarray(velocity, dtype=float)

    def is_stopped(self) -> bool:
        return bool(
            np.linalg.norm(self.target_task_velocity) <= 1e-6
            and np.linalg.norm(self.applied_task_velocity) <= 1e-4
            and np.linalg.norm(self.joint_velocity_deg_s) <= 1e-4
        )

    def _velocity_bounds(
        self,
        joints_deg: np.ndarray,
        dt_s: float,
        limits: CartesianServoLimits,
    ) -> tuple[np.ndarray, np.ndarray]:
        speeds = np.asarray(limits.joint_speed_deg_s, dtype=float)
        lower = -speeds
        upper = speeds
        margin = max(0.0, float(limits.joint_limit_margin_deg))
        for index, joint in enumerate(self.joints):
            safe_min = min(joint.max_deg, joint.min_deg + margin)
            safe_max = max(joint.min_deg, joint.max_deg - margin)
            lower[index] = max(lower[index], (safe_min - joints_deg[index]) / dt_s)
            upper[index] = min(upper[index], (safe_max - joints_deg[index]) / dt_s)
        return lower, upper

    def _solve_at(
        self,
        joints_deg: np.ndarray,
        task_velocity: np.ndarray,
        dt_s: float,
        limits: CartesianServoLimits,
    ) -> tuple[np.ndarray, float, list[int], float]:
        translation_active = float(np.linalg.norm(task_velocity[:3])) > 1e-8
        phi_active = abs(float(task_velocity[3])) > 1e-8
        active_rows: list[int] = []
        if translation_active:
            # Include all three position rows, including commanded zeros, so
            # lateral TCP velocity is constrained to zero.
            active_rows.extend((0, 1, 2))
        if phi_active:
            active_rows.append(3)
        if not active_rows:
            return np.zeros(len(self.joints), dtype=float), 1.0, [], 1.0

        task_jacobian = geometric_task_jacobian(joints_deg.tolist(), self.links)[active_rows, :]
        requested = task_velocity[active_rows]
        lower, upper = self._velocity_bounds(joints_deg, dt_s, limits)
        speed_limits = np.asarray(limits.joint_speed_deg_s, dtype=float)
        centering_velocity = np.zeros(len(self.joints), dtype=float)
        for index, joint in enumerate(self.joints):
            half_range = max(1.0, (joint.max_deg - joint.min_deg) * 0.5)
            center = (joint.min_deg + joint.max_deg) * 0.5
            centering_velocity[index] = (
                0.08 * speed_limits[index] * (center - joints_deg[index]) / half_range
            )
        joint_velocity, scale = solve_direction_preserving_velocity(
            task_jacobian,
            requested,
            lower,
            upper,
            reference_joint_velocity=self.joint_velocity_deg_s + centering_velocity,
        )
        singular_values = np.linalg.svd(task_jacobian, compute_uv=False)
        condition = (
            float(singular_values[0] / singular_values[-1])
            if singular_values.size and singular_values[-1] > 1e-10
            else float("inf")
        )
        return joint_velocity, scale, active_rows, condition

    def _endpoint_escape_velocity(
        self,
        current: np.ndarray,
        current_fk: dict[str, Any],
        task_velocity: np.ndarray,
        dt_s: float,
        limits: CartesianServoLimits,
    ) -> tuple[np.ndarray, float] | None:
        """Use the validated endpoint IK to leave an exact singular pose.

        Differential IK has zero first-order authority in some directions at an
        exact singularity even when a finite move inward is possible. A small
        endpoint lookahead provides a continuous nearby posture. Only a
        speed-bounded fraction is accepted, and only when FK confirms that the
        fraction already moves in the requested Cartesian direction.
        """
        translation_speed = float(np.linalg.norm(task_velocity[:3]))
        if translation_speed <= 1e-8:
            return None
        direction = task_velocity[:3] / translation_speed
        lookahead_mm = min(5.0, max(3.0, translation_speed * dt_s))
        target: dict[str, float | bool] = {
            "x_mm": float(current_fk["x_mm"]) + float(direction[0]) * lookahead_mm,
            "y_mm": float(current_fk["y_mm"]) + float(direction[1]) * lookahead_mm,
            "z_mm": float(current_fk["z_mm"]) + float(direction[2]) * lookahead_mm,
        }
        if abs(float(task_velocity[3])) > 1e-8:
            target["phi_deg"] = float(current_fk["tool_phi_deg"]) + float(task_velocity[3]) * dt_s
        else:
            target["phi_auto"] = True

        endpoint = inverse_kinematics(
            target,
            self.links,
            self.joints,
            current.tolist(),
        )
        selected = endpoint.get("selected") if endpoint.get("ok") else None
        if not isinstance(selected, dict):
            return None
        endpoint_angles = np.asarray(selected.get("angles_deg", []), dtype=float)
        if endpoint_angles.shape != current.shape:
            return None

        joint_delta = endpoint_angles - current
        speed_limits = np.asarray(limits.joint_speed_deg_s, dtype=float)
        fraction = 1.0
        for delta, speed_limit in zip(joint_delta, speed_limits, strict=True):
            if abs(float(delta)) > 1e-10:
                fraction = min(fraction, float(speed_limit) * dt_s / abs(float(delta)))
        if fraction <= 1e-8:
            return None

        candidate = current + joint_delta * fraction
        candidate_fk = forward_kinematics(candidate.tolist(), self.links)
        achieved = _task_delta(current_fk, candidate_fk)[:3]
        achieved_norm = float(np.linalg.norm(achieved))
        progress = float(achieved @ direction)
        alignment = progress / max(achieved_norm, 1e-12)
        if progress <= 1e-6 or alignment < _DIRECTION_TOLERANCE:
            return None

        requested_progress = max(translation_speed * dt_s, 1e-9)
        scale = min(1.0, progress / requested_progress)
        return joint_delta * fraction / dt_s, scale

    def step(self, dt_s: float, limits: CartesianServoLimits) -> dict[str, Any]:
        dt = min(0.1, max(0.005, float(dt_s)))
        if len(limits.joint_speed_deg_s) != len(self.joints):
            raise ValueError("joint speed limit count does not match the robot")

        self.applied_task_velocity[:3] = _ramp_vector(
            self.applied_task_velocity[:3],
            self.target_task_velocity[:3],
            max(1.0, float(limits.tcp_accel_mm_s2)) * dt,
        )
        self.applied_task_velocity[3] = _ramp_scalar(
            float(self.applied_task_velocity[3]),
            float(self.target_task_velocity[3]),
            max(1.0, float(limits.phi_accel_deg_s2)) * dt,
        )

        current = self.commanded_joints_deg.copy()
        current_fk = forward_kinematics(current.tolist(), self.links)
        first_velocity, first_scale, active_rows, condition = self._solve_at(
            current,
            self.applied_task_velocity,
            dt,
            limits,
        )
        midpoint = current + first_velocity * (0.5 * dt)
        midpoint_velocity, midpoint_scale, _, midpoint_condition = self._solve_at(
            midpoint,
            self.applied_task_velocity,
            dt,
            limits,
        )
        scale = min(first_scale, midpoint_scale)
        joint_velocity = midpoint_velocity
        blocked = bool(active_rows and scale < _MIN_USEFUL_SCALE)
        solver_mode = "direction_constrained"
        if blocked:
            joint_velocity = np.zeros(len(self.joints), dtype=float)
            escape = self._endpoint_escape_velocity(
                current,
                current_fk,
                self.applied_task_velocity,
                dt,
                limits,
            )
            if escape is not None:
                joint_velocity, scale = escape
                blocked = False
                solver_mode = "endpoint_singularity_escape"

        target = current + joint_velocity * dt
        for index, joint in enumerate(self.joints):
            target[index] = min(joint.max_deg, max(joint.min_deg, target[index]))
        predicted_fk = forward_kinematics(target.tolist(), self.links)
        achieved_delta = _task_delta(current_fk, predicted_fk)
        requested_delta = self.applied_task_velocity * dt

        requested_position_norm = float(np.linalg.norm(requested_delta[:3]))
        achieved_position_norm = float(np.linalg.norm(achieved_delta[:3]))
        progress = 0.0
        lateral = 0.0
        alignment: float | None = None
        if requested_position_norm > 1e-9:
            requested_direction = requested_delta[:3] / requested_position_norm
            progress = float(achieved_delta[:3] @ requested_direction)
            lateral = float(np.linalg.norm(achieved_delta[:3] - progress * requested_direction))
            alignment = progress / max(achieved_position_norm, 1e-12)

        # A finite integration step can introduce second-order curvature even
        # when instantaneous velocity is exact. Subdivide the joint increment
        # until the actual FK endpoint is aligned with the requested direction.
        if (
            not blocked
            and requested_position_norm > 1e-9
            and (progress <= 0.0 or alignment is None or alignment < _DIRECTION_TOLERANCE)
        ):
            accepted = False
            for reduction in (0.5, 0.25, 0.125, 0.0625):
                reduced_target = current + joint_velocity * dt * reduction
                reduced_fk = forward_kinematics(reduced_target.tolist(), self.links)
                reduced_delta = _task_delta(current_fk, reduced_fk)
                reduced_norm = float(np.linalg.norm(reduced_delta[:3]))
                reduced_progress = float(reduced_delta[:3] @ requested_direction)
                reduced_lateral = float(
                    np.linalg.norm(reduced_delta[:3] - reduced_progress * requested_direction)
                )
                reduced_alignment = reduced_progress / max(reduced_norm, 1e-12)
                if reduced_progress > 0.0 and reduced_alignment >= _DIRECTION_TOLERANCE:
                    target = reduced_target
                    predicted_fk = reduced_fk
                    achieved_delta = reduced_delta
                    joint_velocity = joint_velocity * reduction
                    scale *= reduction
                    progress = reduced_progress
                    lateral = reduced_lateral
                    alignment = reduced_alignment
                    accepted = True
                    break
            if not accepted:
                blocked = True
                target = current
                predicted_fk = current_fk
                achieved_delta = np.zeros(4, dtype=float)
                joint_velocity = np.zeros(len(self.joints), dtype=float)
                scale = 0.0
                progress = 0.0
                lateral = 0.0
                alignment = None

        self.commanded_joints_deg = target
        self.joint_velocity_deg_s = joint_velocity
        if self.is_stopped():
            self.applied_task_velocity[:] = 0.0
            self.joint_velocity_deg_s[:] = 0.0

        failure_code = None
        failure_reason = None
        if blocked:
            failure_code = "direction_unavailable"
            failure_reason = "requested Cartesian direction is unavailable at the current pose or joint limits"

        result = {
            "ok": True,
            "blocked": blocked,
            "failure_code": failure_code,
            "failure_reason": failure_reason,
            "target_angles_deg": target.tolist(),
            "joint_velocity_deg_s": joint_velocity.tolist(),
            "target_task_velocity": self.target_task_velocity.tolist(),
            "applied_task_velocity": self.applied_task_velocity.tolist(),
            "velocity_scale": float(scale),
            "solver_mode": solver_mode,
            "requested_delta": {
                "x_mm": float(requested_delta[0]),
                "y_mm": float(requested_delta[1]),
                "z_mm": float(requested_delta[2]),
                "phi_deg": float(requested_delta[3]),
            },
            "achieved_delta": {
                "x_mm": float(achieved_delta[0]),
                "y_mm": float(achieved_delta[1]),
                "z_mm": float(achieved_delta[2]),
                "phi_deg": float(achieved_delta[3]),
            },
            "position_progress_mm": progress,
            "position_lateral_mm": lateral,
            "position_alignment": alignment,
            "condition": max(condition, midpoint_condition),
            "singularity_warning": not isfinite(condition) or condition > 1e4,
            "predicted_fk": predicted_fk,
            "notes": [
                *(
                    ["endpoint IK singularity escape"]
                    if solver_mode == "endpoint_singularity_escape"
                    else []
                ),
                *(["Cartesian speed scaled by constraints"] if 0.0 < scale < 0.999 else []),
            ],
        }
        self.last_result = result
        return result
