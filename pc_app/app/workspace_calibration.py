from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class FiducialDetection:
    corners: list[np.ndarray]
    ids: np.ndarray | None
    dictionary: str
    mode: str
    visible_ids: list[int]
    rejected_count: int = 0


def _as_int_set(values: Any) -> set[int]:
    if not isinstance(values, (list, tuple, set)):
        return set()
    result: set[int] = set()
    for value in values:
        try:
            result.add(int(value))
        except (TypeError, ValueError):
            continue
    return result


def _visible_ids(ids: np.ndarray | None) -> list[int]:
    if ids is None:
        return []
    return sorted(int(marker_id) for marker_id in ids.reshape(-1))


def dictionary_candidates(settings: dict[str, Any]) -> list[str]:
    primary = str(settings.get("dictionary", "DICT_4X4_50")).upper().strip()
    raw_candidates = settings.get("dictionary_candidates")
    candidates: list[str] = []
    for value in [primary, *(raw_candidates if isinstance(raw_candidates, list) else [])]:
        name = str(value).upper().strip()
        if name and name not in candidates:
            candidates.append(name)
    return candidates or [primary]


def make_aruco_detector(dictionary_name: str) -> cv2.aruco.ArucoDetector:
    name = str(dictionary_name or "DICT_4X4_50").upper()
    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        raise ValueError(f"unknown OpenCV ArUco/AprilTag dictionary {name}")
    parameters = cv2.aruco.DetectorParameters()
    if "APRILTAG" in name and hasattr(cv2.aruco, "CORNER_REFINE_APRILTAG"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    elif hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(dictionary_id),
        parameters,
    )


def _mode_images(gray: np.ndarray, settings: dict[str, Any]) -> list[tuple[str, np.ndarray]]:
    invert_first = bool(settings.get("invert_first", False))
    allow_fallback = bool(settings.get("allow_normal_fallback", True))
    first = ("inverted", 255 - gray) if invert_first else ("normal", gray)
    second = ("normal", gray) if invert_first else ("inverted", 255 - gray)
    return [first, second] if allow_fallback else [first]


def _score_detection(
    ids: np.ndarray | None,
    rejected: list[np.ndarray],
    required_ids: set[int],
    dictionary_index: int,
    mode_index: int,
) -> tuple[int, int, int, int]:
    visible = set(_visible_ids(ids))
    required_seen = len(required_ids.intersection(visible)) if required_ids else len(visible)
    return (
        required_seen,
        len(visible),
        len(rejected),
        -(dictionary_index * 10 + mode_index),
    )


def detect_fiducials(image_bgr: np.ndarray, settings: dict[str, Any]) -> FiducialDetection:
    """Detect configured square fiducials in normal and inverted camera images.

    OpenCV exposes AprilTag families through the ArUco detector API, so this
    helper intentionally treats ArUco and AprilTag dictionaries as one marker
    backend. The caller decides which dictionaries are acceptable.
    """

    gray = (
        cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
        if image_bgr.ndim == 3
        else image_bgr.copy()
    )
    dictionaries = dictionary_candidates(settings)
    modes = _mode_images(gray, settings)
    required_ids = _as_int_set(settings.get("required_ids"))
    best: FiducialDetection | None = None
    best_score = (-1, -1, -1, -10_000)
    last_error: ValueError | None = None
    valid_dictionary_seen = False

    for dictionary_index, dictionary_name in enumerate(dictionaries):
        try:
            detector = make_aruco_detector(dictionary_name)
        except ValueError as exc:
            last_error = exc
            continue
        valid_dictionary_seen = True
        for mode_index, (mode, source) in enumerate(modes):
            corners, ids, rejected = detector.detectMarkers(source)
            normalized = [
                np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
                for marker_corners in corners
            ]
            score = _score_detection(ids, rejected, required_ids, dictionary_index, mode_index)
            if score > best_score:
                best_score = score
                best = FiducialDetection(
                    corners=normalized,
                    ids=ids,
                    dictionary=dictionary_name,
                    mode=mode if _visible_ids(ids) else "none",
                    visible_ids=_visible_ids(ids),
                    rejected_count=len(rejected),
                )

    if not valid_dictionary_seen and last_error is not None:
        raise last_error
    if best is None:
        return FiducialDetection([], None, dictionaries[0], "none", [], 0)
    return best


def marker_centers(corners: list[np.ndarray], ids: np.ndarray | None) -> dict[int, np.ndarray]:
    centers: dict[int, np.ndarray] = {}
    if ids is None:
        return centers
    for marker_corners, marker_id in zip(corners, ids.reshape(-1), strict=True):
        centers[int(marker_id)] = np.mean(marker_corners.reshape(4, 2), axis=0).astype(np.float32)
    return centers


def marker_box_polygon(
    corners: list[np.ndarray],
    ids: np.ndarray | None,
    tag_box_corner_index: dict[str, Any] | dict[int, Any],
) -> np.ndarray | None:
    if ids is None or not corners:
        return None
    selected: dict[int, np.ndarray] = {}
    for marker_corners, marker_id in zip(corners, ids.reshape(-1), strict=True):
        raw_index = tag_box_corner_index.get(str(int(marker_id)), tag_box_corner_index.get(int(marker_id)))
        if raw_index is None:
            continue
        try:
            corner_index = int(raw_index)
        except (TypeError, ValueError):
            continue
        if corner_index < 0 or corner_index > 3:
            continue
        selected[int(marker_id)] = marker_corners.reshape(4, 2)[corner_index].astype(np.float32)
    required = []
    for raw_id in tag_box_corner_index:
        try:
            required.append(int(raw_id))
        except (TypeError, ValueError):
            continue
    if any(marker_id not in selected for marker_id in required):
        return None
    points = np.asarray([selected[marker_id] for marker_id in required], dtype=np.float32)
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    return points[np.argsort(angles)].astype(np.float32)


def _point_list_from_mapping(
    point_mapping: dict[str, Any] | dict[int, Any],
    ids: list[int],
) -> np.ndarray | None:
    points: list[list[float]] = []
    for marker_id in ids:
        value = point_mapping.get(str(marker_id), point_mapping.get(marker_id))
        if not isinstance(value, (list, tuple)) or len(value) < 2:
            return None
        try:
            point = [float(value[0]), float(value[1])]
        except (TypeError, ValueError):
            return None
        if not np.all(np.isfinite(point)):
            return None
        points.append(point)
    return np.asarray(points, dtype=np.float64)


def solve_image_to_robot_homography(
    image_points: np.ndarray,
    robot_points: np.ndarray,
) -> tuple[np.ndarray | None, dict[str, Any]]:
    if image_points.shape[0] < 4 or robot_points.shape[0] < 4:
        return None, {"ok": False, "error": "homography requires at least four point correspondences"}
    method = cv2.RANSAC if image_points.shape[0] > 4 else 0
    homography, inlier_mask = cv2.findHomography(
        image_points.astype(np.float64),
        robot_points.astype(np.float64),
        method,
        2.5,
    )
    if homography is None or homography.shape != (3, 3) or not np.all(np.isfinite(homography)):
        return None, {"ok": False, "error": "OpenCV could not solve image-to-robot homography"}
    mapped = cv2.perspectiveTransform(
        image_points.reshape(-1, 1, 2).astype(np.float64),
        homography,
    ).reshape(-1, 2)
    errors = np.linalg.norm(mapped - robot_points[:, :2], axis=1)
    inliers = int(np.count_nonzero(inlier_mask)) if inlier_mask is not None else int(len(errors))
    return homography, {
        "ok": True,
        "point_count": int(len(errors)),
        "inlier_count": inliers,
        "rmse_mm": float(np.sqrt(np.mean(errors**2))),
        "max_error_mm": float(np.max(errors)),
    }


def homography_from_marker_centers(
    centers: dict[int, np.ndarray],
    settings: dict[str, Any],
) -> tuple[np.ndarray | None, dict[str, Any]]:
    required_ids = [int(value) for value in settings.get("required_ids", [])]
    robot_positions = settings.get("tag_centers_robot_mm")
    if not isinstance(robot_positions, dict) or not required_ids:
        return None, {"ok": False, "error": "workspace tag robot positions are not configured"}
    if any(marker_id not in centers for marker_id in required_ids):
        missing = [marker_id for marker_id in required_ids if marker_id not in centers]
        return None, {"ok": False, "error": f"missing required workspace tags: {missing}"}
    image_points = np.asarray([centers[marker_id] for marker_id in required_ids], dtype=np.float64)
    robot_points = _point_list_from_mapping(robot_positions, required_ids)
    if robot_points is None:
        return None, {"ok": False, "error": "workspace tag robot positions are invalid"}
    homography, metrics = solve_image_to_robot_homography(image_points, robot_points)
    return homography, {**metrics, "tags_used": required_ids}


def scaled_reference_centers(
    settings: dict[str, Any],
    image_shape: tuple[int, ...],
) -> dict[int, np.ndarray]:
    raw_points = settings.get("reference_points_px")
    if not isinstance(raw_points, dict):
        return {}
    resolution = settings.get("reference_resolution")
    if not isinstance(resolution, dict):
        resolution = {}
    reference_width = float(resolution.get("width", image_shape[1]) or image_shape[1])
    reference_height = float(resolution.get("height", image_shape[0]) or image_shape[0])
    scale_x = float(image_shape[1]) / max(reference_width, 1.0)
    scale_y = float(image_shape[0]) / max(reference_height, 1.0)
    points: dict[int, np.ndarray] = {}
    for raw_id, value in raw_points.items():
        try:
            marker_id = int(raw_id)
            point = np.asarray([float(value[0]) * scale_x, float(value[1]) * scale_y], dtype=np.float64)
        except (TypeError, ValueError, IndexError):
            continue
        if np.all(np.isfinite(point)):
            points[marker_id] = point
    return points


def saved_homography(
    settings: dict[str, Any],
    image_shape: tuple[int, ...],
) -> tuple[np.ndarray | None, dict[str, Any]]:
    required_ids = [int(value) for value in settings.get("required_ids", [])]
    references = scaled_reference_centers(settings, image_shape)
    robot_positions = settings.get("tag_centers_robot_mm")
    if not required_ids:
        return None, {"ok": False, "error": "workspace required_ids are not configured"}
    if any(marker_id not in references for marker_id in required_ids):
        missing = [marker_id for marker_id in required_ids if marker_id not in references]
        return None, {"ok": False, "error": f"saved workspace reference points missing tags: {missing}"}
    if not isinstance(robot_positions, dict):
        return None, {"ok": False, "error": "saved workspace robot points are not configured"}
    image_points = np.asarray([references[marker_id] for marker_id in required_ids], dtype=np.float64)
    robot_points = _point_list_from_mapping(robot_positions, required_ids)
    if robot_points is None:
        return None, {"ok": False, "error": "saved workspace robot points are invalid"}
    homography, metrics = solve_image_to_robot_homography(image_points, robot_points)
    return homography, {**metrics, "tags_used": required_ids, "source": "saved_reference_points"}


def layout_diagnostics(
    centers: dict[int, np.ndarray],
    settings: dict[str, Any],
    image_shape: tuple[int, ...],
) -> tuple[list[str], dict[int, float]]:
    """Detect obvious tag-ID swaps without rejecting a uniformly moved camera.

    The old saved pixel centers are used only for pairwise orientation checks.
    That catches tags placed in the wrong physical corner, while still allowing
    live calibration when the whole camera image has shifted or scaled.
    """

    references = scaled_reference_centers(settings, image_shape)
    required = [int(value) for value in settings.get("required_ids", [])]
    visible_required = [marker_id for marker_id in required if marker_id in centers and marker_id in references]
    if len(visible_required) < 3:
        return [], {}

    min_alignment = float(settings.get("min_layout_vector_alignment", 0.2))
    mismatches: list[str] = []
    errors: dict[int, float] = {}
    for first_index, first_id in enumerate(visible_required):
        for second_id in visible_required[first_index + 1 :]:
            live_delta = centers[second_id] - centers[first_id]
            reference_delta = references[second_id] - references[first_id]
            live_norm = float(np.linalg.norm(live_delta))
            reference_norm = float(np.linalg.norm(reference_delta))
            if live_norm < 1e-6 or reference_norm < 1e-6:
                continue
            alignment = float(np.dot(live_delta, reference_delta) / (live_norm * reference_norm))
            if alignment < min_alignment:
                mismatches.append(
                    f"tag vector {first_id}->{second_id} points opposite the saved workspace layout"
                )
                errors[first_id] = max(errors.get(first_id, 0.0), abs(alignment))
                errors[second_id] = max(errors.get(second_id, 0.0), abs(alignment))
    return mismatches, errors


def apply_homography(homography: np.ndarray, image_point: list[float] | tuple[float, float]) -> dict[str, float]:
    point = np.asarray([[[float(image_point[0]), float(image_point[1])]]], dtype=np.float64)
    mapped = cv2.perspectiveTransform(point, homography)[0][0]
    mapped[np.abs(mapped) < 1e-9] = 0.0
    return {"x_mm": float(mapped[0]), "y_mm": float(mapped[1]), "z_mm": 0.0}


def polygon_from_robot(
    homography_image_to_robot: np.ndarray,
    settings: dict[str, Any],
) -> np.ndarray | None:
    polygon = settings.get("workspace_polygon_robot_mm")
    if not isinstance(polygon, list) or len(polygon) < 3:
        return None
    try:
        robot_points = np.asarray([[float(point[0]), float(point[1])] for point in polygon], dtype=np.float64)
    except (TypeError, ValueError, IndexError):
        return None
    if not np.all(np.isfinite(robot_points)):
        return None
    robot_to_image = np.linalg.inv(homography_image_to_robot)
    image_points = cv2.perspectiveTransform(robot_points.reshape(-1, 1, 2), robot_to_image).reshape(-1, 2)
    return image_points.astype(np.float32)


def serialize_homography(homography: np.ndarray) -> list[list[float]]:
    matrix = np.asarray(homography, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("homography must be a finite 3x3 matrix")
    return [[float(value) for value in row] for row in matrix]
