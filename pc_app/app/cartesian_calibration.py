from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
from hashlib import sha256
from math import hypot, isfinite, sqrt
from typing import Any
from uuid import uuid4

import numpy as np

from .config import RobotConfig


SCHEMA_VERSION = 1
SUPPORTED_MODELS = {"constant_xyz", "affine_xy_z_offset"}

DEFAULT_THRESHOLDS: dict[str, float] = {
    "good_xy_rmse_mm": 5.0,
    "acceptable_xy_max_mm": 10.0,
    "good_z_rmse_mm": 3.0,
    "acceptable_z_max_mm": 5.0,
    "warn_xy_rmse_mm": 8.0,
    "warn_xy_max_mm": 15.0,
    "warn_z_rmse_mm": 5.0,
    "warn_z_max_mm": 8.0,
    "minimum_sample_quality": 0.2,
    "maximum_sample_residual_mm": 250.0,
    "outlier_floor_mm": 5.0,
    "outlier_mad_scale": 3.5,
}


def calibration_settings(config: RobotConfig) -> dict[str, Any]:
    defaults: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "enabled": False,
        "active_profile": active_tool_name(config),
        "default_model": "affine_xy_z_offset",
        "thresholds": deepcopy(DEFAULT_THRESHOLDS),
        "profiles": {},
    }
    raw = config.raw.get("kinematics_calibration")
    if not isinstance(raw, dict):
        return defaults
    merged = deepcopy(defaults)
    for key, value in raw.items():
        if key == "thresholds" and isinstance(value, dict):
            merged["thresholds"].update(deepcopy(value))
        elif key == "profiles" and isinstance(value, dict):
            merged["profiles"] = deepcopy(value)
        else:
            merged[key] = deepcopy(value)
    return merged


def active_tool_name(config: RobotConfig) -> str:
    tools = config.raw.get("tools")
    if isinstance(tools, dict):
        return str(tools.get("active") or "gripper")
    return "gripper"


def workspace_context(config: RobotConfig) -> dict[str, Any]:
    camera = config.raw.get("camera")
    calibration = camera.get("calibration") if isinstance(camera, dict) else None
    workspace = calibration.get("workspace_aruco") if isinstance(calibration, dict) else None
    if not isinstance(workspace, dict):
        return {
            "calibrated": False,
            "source": "none",
            "signature": "",
            "last_calibrated_at": None,
            "message": "no planar workspace calibration is configured",
        }
    reference = {
        "reference_points_px": workspace.get("reference_points_px"),
        "reference_workspace_corners_px": workspace.get("reference_workspace_corners_px"),
        "reference_resolution": workspace.get("reference_resolution"),
        "workspace_polygon_robot_mm": workspace.get("workspace_polygon_robot_mm"),
        "last_calibrated_at": workspace.get("last_calibrated_at"),
    }
    signature = sha256(
        repr(reference).encode("utf-8")
    ).hexdigest()[:16]
    has_reference = bool(workspace.get("reference_points_px"))
    return {
        "calibrated": bool(workspace.get("enabled", True) and has_reference),
        "source": "workspace_aruco_saved" if has_reference else "workspace_aruco_unavailable",
        "signature": signature if has_reference else "",
        "last_calibrated_at": workspace.get("last_calibrated_at"),
        "message": (
            "saved planar workspace calibration is available"
            if has_reference
            else "planar workspace mapping has no saved reference points"
        ),
    }


def _profile_key(settings: dict[str, Any], config: RobotConfig, requested: str | None = None) -> str:
    if requested:
        return str(requested)
    active = str(settings.get("active_profile") or "").strip()
    tool = active_tool_name(config)
    if active and active == tool:
        return active
    return tool


def active_profile(config: RobotConfig, requested: str | None = None) -> tuple[str, dict[str, Any] | None]:
    settings = calibration_settings(config)
    key = _profile_key(settings, config, requested)
    profiles = settings.get("profiles")
    profile = profiles.get(key) if isinstance(profiles, dict) else None
    return key, profile if isinstance(profile, dict) else None


def _target_xyz(target: dict[str, Any]) -> np.ndarray:
    values = np.array(
        [
            float(target.get("x_mm", target.get("x"))),
            float(target.get("y_mm", target.get("y"))),
            float(target.get("z_mm", target.get("z"))),
        ],
        dtype=float,
    )
    if not np.all(np.isfinite(values)):
        raise ValueError("Cartesian target must contain finite x_mm, y_mm, and z_mm values")
    return values


def _model_coefficients(profile: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, float]:
    result = profile.get("result")
    if not isinstance(result, dict):
        raise ValueError("calibration profile has no fitted result")
    coefficients = result.get("coefficients")
    if not isinstance(coefficients, dict):
        raise ValueError("calibration result has no coefficients")
    matrix = np.asarray(coefficients.get("xy_matrix"), dtype=float)
    offset = np.asarray(coefficients.get("xy_offset_mm"), dtype=float)
    z_offset = float(coefficients.get("z_offset_mm"))
    if matrix.shape != (2, 2) or offset.shape != (2,):
        raise ValueError("calibration XY coefficients have invalid dimensions")
    if not np.all(np.isfinite(matrix)) or not np.all(np.isfinite(offset)) or not isfinite(z_offset):
        raise ValueError("calibration coefficients contain non-finite values")
    if abs(float(np.linalg.det(matrix))) < 1e-8:
        raise ValueError("calibration XY transform is singular")
    return matrix, offset, z_offset


def predict_physical_pose(
    model_pose: dict[str, Any],
    config: RobotConfig,
    *,
    profile_key: str | None = None,
    require_enabled: bool = True,
) -> dict[str, Any]:
    settings = calibration_settings(config)
    key, profile = active_profile(config, profile_key)
    pose = deepcopy(model_pose)
    if (
        profile is None
        or (require_enabled and not bool(settings.get("enabled", False)))
        or (require_enabled and not bool(profile.get("enabled", True)))
    ):
        return pose
    matrix, offset, z_offset = _model_coefficients(profile)
    xyz = _target_xyz(pose)
    predicted_xy = matrix @ xyz[:2] + offset
    pose["x_mm"] = float(predicted_xy[0])
    pose["y_mm"] = float(predicted_xy[1])
    pose["z_mm"] = float(xyz[2] + z_offset)
    pose["calibration_profile"] = key
    return pose


def correct_cartesian_target(
    target: dict[str, Any],
    config: RobotConfig,
    *,
    apply_enabled: bool = True,
    profile_key: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    requested = deepcopy(target)
    command = deepcopy(target)
    settings = calibration_settings(config)
    key, profile = active_profile(config, profile_key)
    metadata: dict[str, Any] = {
        "applied": False,
        "profile_key": key,
        "model_type": profile.get("model_type") if isinstance(profile, dict) else None,
        "requested_target": requested,
        "command_target": command,
        "warnings": [],
    }
    if not apply_enabled:
        metadata["reason"] = "disabled_for_request"
        return command, metadata
    if not bool(settings.get("enabled", False)):
        metadata["reason"] = "disabled"
        return command, metadata
    if not isinstance(profile, dict):
        metadata["reason"] = "profile_missing"
        return command, metadata
    if not bool(profile.get("enabled", True)):
        metadata["reason"] = "profile_disabled"
        return command, metadata
    try:
        matrix, offset, z_offset = _model_coefficients(profile)
        desired = _target_xyz(requested)
        command_xy = np.linalg.solve(matrix, desired[:2] - offset)
        command["x_mm"] = float(command_xy[0])
        command["y_mm"] = float(command_xy[1])
        command["z_mm"] = float(desired[2] - z_offset)
    except (TypeError, ValueError, np.linalg.LinAlgError) as exc:
        metadata["reason"] = "invalid_result"
        metadata["warnings"].append(str(exc))
        return deepcopy(target), metadata

    current_workspace = workspace_context(config)
    fitted_workspace = profile.get("workspace")
    if (
        isinstance(fitted_workspace, dict)
        and fitted_workspace.get("signature")
        and fitted_workspace.get("signature") != current_workspace.get("signature")
    ):
        metadata["warnings"].append(
            "workspace calibration differs from the map used when this TCP calibration was fitted"
        )
    metadata.update(
        {
            "applied": True,
            "reason": "enabled",
            "command_target": deepcopy(command),
            "result_id": profile.get("result", {}).get("id"),
        }
    )
    return command, metadata


def correct_waypoint_program(
    waypoints: list[dict[str, Any]],
    config: RobotConfig,
    *,
    apply_enabled: bool = True,
) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    corrected: list[dict[str, Any]] = []
    metadata: list[dict[str, Any]] = []
    for index, waypoint in enumerate(waypoints):
        item = deepcopy(waypoint)
        kind = str(item.get("type") or item.get("kind") or "cartesian").lower()
        if kind != "cartesian":
            corrected.append(item)
            continue
        raw_target = item.get("target") if isinstance(item.get("target"), dict) else item
        command_target, correction = correct_cartesian_target(
            raw_target,
            config,
            apply_enabled=apply_enabled,
        )
        correction["waypoint_index"] = index
        correction["label"] = item.get("label") or item.get("name") or f"waypoint {index + 1}"
        metadata.append(correction)
        if isinstance(item.get("target"), dict):
            item["target"] = command_target
        else:
            item.update(command_target)
        corrected.append(item)
    return corrected, metadata


def _finite_vector(value: Any, count: int, name: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{name} must contain exactly {count} values")
    result = [float(item) for item in value]
    if not all(isfinite(item) for item in result):
        raise ValueError(f"{name} must contain only finite values")
    return result


def create_sample(
    payload: dict[str, Any],
    config: RobotConfig,
    reported_joints_deg: list[float],
    fk_predicted: dict[str, Any],
) -> dict[str, Any]:
    intended = deepcopy(payload.get("intended_target"))
    command = deepcopy(payload.get("command_target") or intended)
    measured = deepcopy(payload.get("measured"))
    if not isinstance(intended, dict):
        raise ValueError("intended_target is required")
    if not isinstance(command, dict):
        raise ValueError("command_target is required")
    if not isinstance(measured, dict):
        raise ValueError("measured is required")
    intended_xyz = _target_xyz(intended)
    command_xyz = _target_xyz(command)
    fk_xyz = _target_xyz(fk_predicted)
    measured_xyz = _target_xyz(measured)
    joints = _finite_vector(reported_joints_deg, 4, "reported_joints_deg")
    quality = float(payload.get("quality", 1.0))
    thresholds = calibration_settings(config).get("thresholds", DEFAULT_THRESHOLDS)
    minimum_quality = float(thresholds.get("minimum_sample_quality", 0.2))
    if not isfinite(quality) or not 0.0 <= quality <= 1.0:
        raise ValueError("quality must be between 0 and 1")
    if quality < minimum_quality:
        raise ValueError(f"sample quality {quality:.2f} is below the minimum {minimum_quality:.2f}")
    role = str(payload.get("role") or "fit").lower()
    if role not in {"fit", "validation"}:
        raise ValueError("sample role must be fit or validation")
    model_residual = measured_xyz - fk_xyz
    maximum_residual = float(thresholds.get("maximum_sample_residual_mm", 250.0))
    if float(np.linalg.norm(model_residual)) > maximum_residual:
        raise ValueError(
            f"measured TCP differs from FK by more than {maximum_residual:.1f} mm; verify frame, units, and marker"
        )
    intended_phi = intended.get("phi_deg", intended.get("phi"))
    command_phi = command.get("phi_deg", command.get("phi"))
    sample = {
        "id": str(payload.get("id") or uuid4()),
        "role": role,
        "timestamp": str(payload.get("timestamp") or datetime.now(timezone.utc).isoformat()),
        "tool": active_tool_name(config),
        "workspace": workspace_context(config),
        "intended_target": {
            "x_mm": float(intended_xyz[0]),
            "y_mm": float(intended_xyz[1]),
            "z_mm": float(intended_xyz[2]),
            "phi_deg": float(intended_phi) if intended_phi is not None else None,
        },
        "command_target": {
            "x_mm": float(command_xyz[0]),
            "y_mm": float(command_xyz[1]),
            "z_mm": float(command_xyz[2]),
            "phi_deg": float(command_phi) if command_phi is not None else None,
        },
        "reported_joints_deg": joints,
        "joint_source": str(payload.get("joint_source") or "reported"),
        "fk_predicted": {
            "x_mm": float(fk_xyz[0]),
            "y_mm": float(fk_xyz[1]),
            "z_mm": float(fk_xyz[2]),
            "phi_deg": float(fk_predicted.get("tool_phi_deg", fk_predicted.get("phi_deg", 0.0))),
        },
        "measured": {
            "x_mm": float(measured_xyz[0]),
            "y_mm": float(measured_xyz[1]),
            "z_mm": float(measured_xyz[2]),
        },
        "measurement_source": deepcopy(payload.get("measurement_source") or {}),
        "quality": quality,
        "notes": str(payload.get("notes") or ""),
        "residuals": {
            "model_mm": {
                "x": float(model_residual[0]),
                "y": float(model_residual[1]),
                "z": float(model_residual[2]),
                "xy": float(hypot(model_residual[0], model_residual[1])),
            },
            "command_mm": {
                "x": float(measured_xyz[0] - command_xyz[0]),
                "y": float(measured_xyz[1] - command_xyz[1]),
                "z": float(measured_xyz[2] - command_xyz[2]),
            },
            "ik_target_mm": {
                "x": float(fk_xyz[0] - command_xyz[0]),
                "y": float(fk_xyz[1] - command_xyz[1]),
                "z": float(fk_xyz[2] - command_xyz[2]),
                "xyz": float(np.linalg.norm(fk_xyz - command_xyz)),
            },
            "landing_mm": {
                "x": float(measured_xyz[0] - intended_xyz[0]),
                "y": float(measured_xyz[1] - intended_xyz[1]),
                "z": float(measured_xyz[2] - intended_xyz[2]),
                "xy": float(hypot(measured_xyz[0] - intended_xyz[0], measured_xyz[1] - intended_xyz[1])),
            },
        },
    }
    return sample


def _sample_arrays(samples: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    expected = np.array(
        [
            [
                float(sample["fk_predicted"]["x_mm"]),
                float(sample["fk_predicted"]["y_mm"]),
                float(sample["fk_predicted"]["z_mm"]),
            ]
            for sample in samples
        ],
        dtype=float,
    )
    measured = np.array(
        [
            [
                float(sample["measured"]["x_mm"]),
                float(sample["measured"]["y_mm"]),
                float(sample["measured"]["z_mm"]),
            ]
            for sample in samples
        ],
        dtype=float,
    )
    quality = np.array([max(0.01, float(sample.get("quality", 1.0))) for sample in samples], dtype=float)
    return expected, measured, quality


def _solve_model(
    samples: list[dict[str, Any]],
    model_type: str,
) -> tuple[np.ndarray, np.ndarray, float]:
    expected, measured, quality = _sample_arrays(samples)
    weights = np.sqrt(quality)[:, None]
    if model_type == "constant_xyz":
        residual = measured - expected
        weighted = residual * quality[:, None]
        offset_xyz = weighted.sum(axis=0) / max(float(quality.sum()), 1e-9)
        return np.identity(2), offset_xyz[:2], float(offset_xyz[2])
    if model_type != "affine_xy_z_offset":
        raise ValueError(f"unsupported calibration model {model_type}")
    design = np.column_stack((expected[:, 0], expected[:, 1], np.ones(len(samples))))
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError("affine XY fitting requires samples spanning at least three non-collinear XY locations")
    weighted_design = design * weights
    solution_x, *_ = np.linalg.lstsq(weighted_design, measured[:, 0:1] * weights, rcond=None)
    solution_y, *_ = np.linalg.lstsq(weighted_design, measured[:, 1:2] * weights, rcond=None)
    matrix = np.array(
        [
            [float(solution_x[0, 0]), float(solution_x[1, 0])],
            [float(solution_y[0, 0]), float(solution_y[1, 0])],
        ]
    )
    offset = np.array([float(solution_x[2, 0]), float(solution_y[2, 0])])
    z_residual = measured[:, 2] - expected[:, 2]
    z_offset = float(np.average(z_residual, weights=quality))
    condition = float(np.linalg.cond(matrix))
    if not isfinite(condition) or condition > 100.0 or abs(float(np.linalg.det(matrix))) < 1e-5:
        raise ValueError("fitted affine XY correction is ill-conditioned; collect wider, non-collinear samples")
    return matrix, offset, z_offset


def _predict_array(expected: np.ndarray, matrix: np.ndarray, offset: np.ndarray, z_offset: float) -> np.ndarray:
    predicted = np.empty_like(expected)
    predicted[:, :2] = expected[:, :2] @ matrix.T + offset
    predicted[:, 2] = expected[:, 2] + z_offset
    return predicted


def _metrics(vectors: np.ndarray) -> dict[str, Any]:
    if vectors.size == 0:
        return {
            "count": 0,
            "xy_rmse_mm": None,
            "xy_max_mm": None,
            "z_rmse_mm": None,
            "z_max_abs_mm": None,
            "xyz_rmse_mm": None,
            "worst_samples": [],
        }
    xy = np.linalg.norm(vectors[:, :2], axis=1)
    z = np.abs(vectors[:, 2])
    xyz = np.linalg.norm(vectors, axis=1)
    return {
        "count": int(len(vectors)),
        "xy_rmse_mm": float(sqrt(float(np.mean(xy**2)))),
        "xy_max_mm": float(np.max(xy)),
        "z_rmse_mm": float(sqrt(float(np.mean(z**2)))),
        "z_max_abs_mm": float(np.max(z)),
        "xyz_rmse_mm": float(sqrt(float(np.mean(xyz**2)))),
    }


def _quality_status(metrics: dict[str, Any], thresholds: dict[str, Any]) -> str:
    if not metrics.get("count"):
        return "not_run"
    good = (
        float(metrics["xy_rmse_mm"]) <= float(thresholds.get("good_xy_rmse_mm", 5.0))
        and float(metrics["xy_max_mm"]) <= float(thresholds.get("acceptable_xy_max_mm", 10.0))
        and float(metrics["z_rmse_mm"]) <= float(thresholds.get("good_z_rmse_mm", 3.0))
        and float(metrics["z_max_abs_mm"]) <= float(thresholds.get("acceptable_z_max_mm", 5.0))
    )
    if good:
        return "pass"
    warning = (
        float(metrics["xy_rmse_mm"]) <= float(thresholds.get("warn_xy_rmse_mm", 8.0))
        and float(metrics["xy_max_mm"]) <= float(thresholds.get("warn_xy_max_mm", 15.0))
        and float(metrics["z_rmse_mm"]) <= float(thresholds.get("warn_z_rmse_mm", 5.0))
        and float(metrics["z_max_abs_mm"]) <= float(thresholds.get("warn_z_max_mm", 8.0))
    )
    return "warn" if warning else "fail"


def _outlier_mask(
    expected: np.ndarray,
    measured: np.ndarray,
    matrix: np.ndarray,
    offset: np.ndarray,
    z_offset: float,
    thresholds: dict[str, Any],
) -> np.ndarray:
    residual = measured - _predict_array(expected, matrix, offset, z_offset)
    scalar = np.linalg.norm(residual, axis=1)
    median = float(np.median(scalar))
    mad = float(np.median(np.abs(scalar - median)))
    robust_sigma = 1.4826 * mad
    limit = max(
        float(thresholds.get("outlier_floor_mm", 5.0)),
        median + float(thresholds.get("outlier_mad_scale", 3.5)) * robust_sigma,
    )
    return scalar <= limit


def _sample_diagnostics(
    fit_samples: list[dict[str, Any]],
    matrix: np.ndarray,
    offset: np.ndarray,
    z_offset: float,
) -> list[str]:
    notes: list[str] = []
    model_residuals = np.array(
        [
            [
                float(sample["residuals"]["model_mm"]["x"]),
                float(sample["residuals"]["model_mm"]["y"]),
                float(sample["residuals"]["model_mm"]["z"]),
            ]
            for sample in fit_samples
        ],
        dtype=float,
    )
    ik_errors = [
        float(sample.get("residuals", {}).get("ik_target_mm", {}).get("xyz", 0.0))
        for sample in fit_samples
    ]
    if ik_errors and max(ik_errors) > 2.0:
        notes.append(
            "some samples have FK-to-command error above 2 mm; inspect IK reachability, joint tracking, or reported-angle quality separately"
        )
    mean_offset = np.mean(model_residuals, axis=0)
    spread = np.std(model_residuals, axis=0)
    if float(np.linalg.norm(mean_offset)) > max(3.0, float(np.linalg.norm(spread)) * 1.5):
        notes.append("residuals are dominated by a consistent offset, which is compatible with TCP or zero-offset error")
    affine_delta = float(np.linalg.norm(matrix - np.identity(2)))
    if affine_delta > 0.04:
        notes.append(
            "the XY fit includes noticeable scale/skew; verify workspace calibration and geometry before treating it as robot-only error"
        )
    if abs(z_offset) > 10.0:
        notes.append("large fitted Z offset suggests TCP length, touch-off reference, or joint-zero error")
    if len(fit_samples) >= 2:
        expected = np.array(
            [
                [sample["fk_predicted"]["x_mm"], sample["fk_predicted"]["y_mm"]]
                for sample in fit_samples
            ],
            dtype=float,
        )
        measured = np.array(
            [[sample["measured"]["x_mm"], sample["measured"]["y_mm"]] for sample in fit_samples],
            dtype=float,
        )
        repeatability_spreads: list[float] = []
        for index in range(len(fit_samples)):
            neighbours = np.linalg.norm(expected - expected[index], axis=1) <= 3.0
            if int(np.count_nonzero(neighbours)) >= 2:
                group = measured[neighbours]
                repeatability_spreads.append(
                    float(np.max(np.linalg.norm(group - np.mean(group, axis=0), axis=1)))
                )
        if repeatability_spreads and max(repeatability_spreads) > 3.0:
            notes.append(
                "repeated nearby poses vary by more than 3 mm; backlash, compliance, or measurement repeatability may limit calibration"
            )
    return notes


def _evaluate_samples(
    samples: list[dict[str, Any]],
    matrix: np.ndarray,
    offset: np.ndarray,
    z_offset: float,
    thresholds: dict[str, Any],
) -> dict[str, Any]:
    if not samples:
        empty = _metrics(np.empty((0, 3)))
        return {"before": empty, "after_model": empty, "landing": empty, "status": "not_run", "worst_samples": []}
    expected, measured, _ = _sample_arrays(samples)
    intended = np.array(
        [
            [
                float(sample["intended_target"]["x_mm"]),
                float(sample["intended_target"]["y_mm"]),
                float(sample["intended_target"]["z_mm"]),
            ]
            for sample in samples
        ],
        dtype=float,
    )
    before_vectors = measured - expected
    after_vectors = measured - _predict_array(expected, matrix, offset, z_offset)
    landing_vectors = measured - intended
    after_norm = np.linalg.norm(after_vectors, axis=1)
    worst_indices = np.argsort(after_norm)[::-1][:5]
    worst = [
        {
            "id": samples[int(index)].get("id"),
            "role": samples[int(index)].get("role"),
            "error_xyz_mm": float(after_norm[int(index)]),
            "error_xy_mm": float(np.linalg.norm(after_vectors[int(index), :2])),
            "error_z_mm": float(after_vectors[int(index), 2]),
        }
        for index in worst_indices
    ]
    after_metrics = _metrics(after_vectors)
    return {
        "before": _metrics(before_vectors),
        "after_model": after_metrics,
        "landing": _metrics(landing_vectors),
        "status": _quality_status(after_metrics, thresholds),
        "worst_samples": worst,
    }


def fit_profile(
    settings: dict[str, Any],
    config: RobotConfig,
    *,
    profile_key: str | None = None,
    model_type: str | None = None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    updated = deepcopy(settings)
    key = _profile_key(updated, config, profile_key)
    profiles = updated.setdefault("profiles", {})
    profile = deepcopy(profiles.get(key) or {})
    samples = profile.get("samples")
    if not isinstance(samples, list):
        samples = []
    chosen_model = str(model_type or profile.get("model_type") or updated.get("default_model") or "affine_xy_z_offset")
    if chosen_model not in SUPPORTED_MODELS:
        raise ValueError(f"model_type must be one of {sorted(SUPPORTED_MODELS)}")
    fit_samples = [sample for sample in samples if isinstance(sample, dict) and sample.get("role", "fit") == "fit"]
    minimum = 2 if chosen_model == "constant_xyz" else 4
    if len(fit_samples) < minimum:
        raise ValueError(f"{chosen_model} requires at least {minimum} fit samples")
    thresholds = updated.get("thresholds") if isinstance(updated.get("thresholds"), dict) else deepcopy(DEFAULT_THRESHOLDS)

    inliers = list(range(len(fit_samples)))
    rejected: list[int] = []
    for _ in range(3):
        active_samples = [fit_samples[index] for index in inliers]
        matrix, offset, z_offset = _solve_model(active_samples, chosen_model)
        expected, measured, _ = _sample_arrays(active_samples)
        local_mask = _outlier_mask(expected, measured, matrix, offset, z_offset, thresholds)
        next_inliers = [index for index, keep in zip(inliers, local_mask, strict=True) if bool(keep)]
        next_rejected = [index for index, keep in zip(inliers, local_mask, strict=True) if not bool(keep)]
        if len(next_inliers) < minimum or not next_rejected:
            break
        rejected.extend(next_rejected)
        inliers = next_inliers

    active_samples = [fit_samples[index] for index in inliers]
    matrix, offset, z_offset = _solve_model(active_samples, chosen_model)
    fit_evaluation = _evaluate_samples(active_samples, matrix, offset, z_offset, thresholds)
    validation_samples = [
        sample for sample in samples if isinstance(sample, dict) and sample.get("role") == "validation"
    ]
    validation = _evaluate_samples(validation_samples, matrix, offset, z_offset, thresholds)
    diagnostics = _sample_diagnostics(active_samples, matrix, offset, z_offset)
    rejected_ids = [fit_samples[index].get("id") for index in sorted(set(rejected))]
    result = {
        "id": str(uuid4()),
        "fitted_at": datetime.now(timezone.utc).isoformat(),
        "model_type": chosen_model,
        "coordinate_frame": "robot base frame; millimetres; +Z upward",
        "fit_sample_count": len(active_samples),
        "validation_sample_count": len(validation_samples),
        "rejected_sample_ids": rejected_ids,
        "coefficients": {
            "xy_matrix": matrix.tolist(),
            "xy_offset_mm": offset.tolist(),
            "z_offset_mm": float(z_offset),
        },
        "fit": fit_evaluation,
        "validation": validation,
        "diagnostics": diagnostics,
    }
    profile.update(
        {
            "tool": key,
            "enabled": bool(profile.get("enabled", False)),
            "model_type": chosen_model,
            "workspace": workspace_context(config),
            "samples": samples,
            "result": result,
        }
    )
    profiles[key] = profile
    updated["schema_version"] = SCHEMA_VERSION
    updated["active_profile"] = key
    return updated, result


def calibration_summary(config: RobotConfig) -> dict[str, Any]:
    settings = calibration_settings(config)
    key, profile = active_profile(config)
    result = profile.get("result") if isinstance(profile, dict) else None
    return {
        "settings": settings,
        "active_profile_key": key,
        "active_profile": deepcopy(profile),
        "enabled": bool(
            settings.get("enabled", False)
            and isinstance(profile, dict)
            and profile.get("enabled", True)
            and isinstance(result, dict)
        ),
        "workspace": workspace_context(config),
        "supported_models": sorted(SUPPORTED_MODELS),
        "thresholds": deepcopy(settings.get("thresholds") or DEFAULT_THRESHOLDS),
        "fit_quality": deepcopy(result.get("fit")) if isinstance(result, dict) else None,
        "validation_quality": deepcopy(result.get("validation")) if isinstance(result, dict) else None,
    }
