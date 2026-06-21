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


class WorkspaceCalibrationSession:
    """Accumulate stable 2D workspace fiducial observations."""

    def __init__(self, settings: dict[str, Any]) -> None:
        self.settings: dict[str, Any] = {}
        self._signature = ""
        self._frame_count = 0
        self._image_shape: tuple[int, int] | None = None
        self._dictionary = ""
        self._mode = "none"
        self._center_samples: dict[int, list[np.ndarray]] = {}
        self._corner_samples: dict[int, list[np.ndarray]] = {}
        self.configure(settings, preserve_frames=False)

    def configure(self, settings: dict[str, Any], preserve_frames: bool = True) -> None:
        signature = repr(
            (
                settings.get("required_ids"),
                settings.get("tag_centers_robot_mm"),
                settings.get("tag_box_corner_index"),
                settings.get("workspace_polygon_robot_mm"),
                settings.get("dictionary"),
                settings.get("dictionary_candidates"),
            )
        )
        changed = signature != self._signature
        self.settings = dict(settings)
        self._signature = signature
        if changed or not preserve_frames:
            self.reset()

    def reset(self) -> None:
        self._frame_count = 0
        self._image_shape = None
        self._dictionary = str(self.settings.get("dictionary", "DICT_4X4_50"))
        self._mode = "none"
        self._center_samples = {}
        self._corner_samples = {}

    @property
    def minimum_samples(self) -> int:
        return max(1, int(self.settings.get("minimum_samples", 12)))

    def add(self, detection: FiducialDetection, image_shape: tuple[int, ...]) -> None:
        normalized_shape = (int(image_shape[0]), int(image_shape[1]))
        if self._image_shape is not None and normalized_shape != self._image_shape:
            self.reset()
        self._image_shape = normalized_shape
        self._frame_count += 1
        self._dictionary = detection.dictionary
        self._mode = detection.mode

        centers = marker_centers(detection.corners, detection.ids)
        corners = marker_box_corners(
            detection.corners,
            detection.ids,
            self.settings.get("tag_box_corner_index", {}),
        )
        required = {int(value) for value in self.settings.get("required_ids", [])}
        for marker_id, point in centers.items():
            if marker_id in required:
                self._center_samples.setdefault(marker_id, []).append(point.astype(np.float64))
        for marker_id, point in corners.items():
            if marker_id in required:
                self._corner_samples.setdefault(marker_id, []).append(point.astype(np.float64))

    def _median_points(self, samples: dict[int, list[np.ndarray]]) -> dict[int, np.ndarray]:
        return {
            marker_id: np.median(np.asarray(points, dtype=np.float64), axis=0)
            for marker_id, points in samples.items()
            if points
        }

    def summary(self) -> dict[str, Any]:
        required = [int(value) for value in self.settings.get("required_ids", [])]
        center_counts = {
            str(marker_id): len(self._center_samples.get(marker_id, []))
            for marker_id in required
        }
        corner_counts = {
            str(marker_id): len(self._corner_samples.get(marker_id, []))
            for marker_id in required
        }
        ready = bool(required) and all(
            center_counts[str(marker_id)] >= self.minimum_samples
            and corner_counts[str(marker_id)] >= self.minimum_samples
            for marker_id in required
        )
        return {
            "frame_count": self._frame_count,
            "minimum_samples": self.minimum_samples,
            "required_ids": required,
            "tag_ids": sorted(self._center_samples),
            "tag_observation_counts": center_counts,
            "corner_observation_counts": corner_counts,
            "image_size_px": (
                {"width": self._image_shape[1], "height": self._image_shape[0]}
                if self._image_shape
                else None
            ),
            "dictionary": self._dictionary,
            "detection_mode": self._mode,
            "ready": ready,
        }

    def solve(self, require_minimum_samples: bool = False) -> dict[str, Any]:
        summary = self.summary()
        required = summary["required_ids"]
        centers = self._median_points(self._center_samples)
        corners = self._median_points(self._corner_samples)
        missing_centers = [marker_id for marker_id in required if marker_id not in centers]
        missing_corners = [marker_id for marker_id in required if marker_id not in corners]
        if missing_centers or missing_corners:
            return {
                **summary,
                "ok": False,
                "ready": False,
                "error": (
                    f"missing tag centers {missing_centers or 'none'}; "
                    f"missing outer workspace corners {missing_corners or 'none'}"
                ),
            }
        if require_minimum_samples and not summary["ready"]:
            return {
                **summary,
                "ok": False,
                "ready": False,
                "error": (
                    f"each workspace tag needs {self.minimum_samples} stable observations "
                    "of both its center and outer corner"
                ),
            }

        homography, metrics = homography_from_marker_centers(centers, self.settings, corners)
        if homography is None:
            return {**summary, **metrics, "ok": False, "ready": False}
        expected_centers = self.settings.get("tag_centers_robot_mm")
        center_errors: list[float] = []
        if isinstance(expected_centers, dict):
            for marker_id in required:
                expected = expected_centers.get(
                    str(marker_id),
                    expected_centers.get(marker_id),
                )
                if not isinstance(expected, (list, tuple)) or len(expected) < 2:
                    continue
                mapped = apply_homography(homography, centers[marker_id])
                center_errors.append(
                    float(
                        np.linalg.norm(
                            np.asarray(
                                [mapped["x_mm"], mapped["y_mm"]],
                                dtype=np.float64,
                            )
                            - np.asarray(expected[:2], dtype=np.float64)
                        )
                    )
                )
        metrics = {
            **metrics,
            "tag_center_rmse_mm": (
                float(np.sqrt(np.mean(np.asarray(center_errors) ** 2)))
                if center_errors
                else None
            ),
            "tag_center_max_error_mm": (
                float(np.max(center_errors)) if center_errors else None
            ),
        }
        max_rmse = float(self.settings.get("max_calibration_rmse_mm", 3.0))
        max_error = float(self.settings.get("max_calibration_error_mm", 7.0))
        max_center_error = float(
            self.settings.get("max_calibration_tag_center_error_mm", 12.0)
        )
        quality_ok = bool(
            float(metrics.get("rmse_mm", float("inf"))) <= max_rmse
            and float(metrics.get("max_error_mm", float("inf"))) <= max_error
            and (
                metrics["tag_center_max_error_mm"] is None
                or float(metrics["tag_center_max_error_mm"]) <= max_center_error
            )
        )
        if require_minimum_samples and not quality_ok:
            return {
                **summary,
                "ok": False,
                "ready": False,
                "error": (
                    f"workspace fit is too inaccurate: {metrics.get('rmse_mm', 0):.2f} mm RMSE, "
                    f"{metrics.get('max_error_mm', 0):.2f} mm max; "
                    f"tag-center max {metrics.get('tag_center_max_error_mm', 0):.2f} mm; "
                    f"limits are {max_rmse:.2f} mm, {max_error:.2f} mm, "
                    f"and {max_center_error:.2f} mm"
                ),
                "metrics": metrics,
                "quality_ok": False,
            }
        return {
            **summary,
            "ok": True,
            "ready": bool(summary["ready"] and quality_ok),
            "sample_ready": bool(summary["ready"]),
            "quality_ok": quality_ok,
            "homography": serialize_homography(homography),
            "reference_points_px": {
                str(marker_id): [float(value) for value in centers[marker_id]]
                for marker_id in required
            },
            "reference_workspace_corners_px": {
                str(marker_id): [float(value) for value in corners[marker_id]]
                for marker_id in required
            },
            "reference_resolution": summary["image_size_px"],
            "dictionary": self._dictionary,
            "detection_mode": self._mode,
            "metrics": metrics,
        }


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


def make_aruco_detector(
    dictionary_name: str,
    settings: dict[str, Any] | None = None,
) -> cv2.aruco.ArucoDetector:
    name = str(dictionary_name or "DICT_4X4_50").upper()
    dictionary_id = getattr(cv2.aruco, name, None)
    if dictionary_id is None:
        raise ValueError(f"unknown OpenCV ArUco/AprilTag dictionary {name}")
    settings = settings or {}
    parameters = cv2.aruco.DetectorParameters()
    parameters.adaptiveThreshWinSizeMax = int(
        settings.get("adaptive_thresh_window_max_px", 53)
    )
    parameters.adaptiveThreshWinSizeStep = int(
        settings.get("adaptive_thresh_window_step_px", 4)
    )
    parameters.minMarkerPerimeterRate = float(
        settings.get("min_marker_perimeter_rate", 0.01)
    )
    parameters.minCornerDistanceRate = float(
        settings.get("min_corner_distance_rate", 0.01)
    )
    parameters.minMarkerDistanceRate = float(
        settings.get("min_marker_distance_rate", 0.03)
    )
    parameters.errorCorrectionRate = float(
        settings.get("error_correction_rate", 0.9)
    )
    parameters.minDistanceToBorder = int(settings.get("min_distance_to_border_px", 1))
    if "APRILTAG" in name and hasattr(cv2.aruco, "CORNER_REFINE_APRILTAG"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
    elif hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(dictionary_id),
        parameters,
    )


def _detector_setting_profiles(settings: dict[str, Any]) -> list[dict[str, Any]]:
    profiles = [dict(settings)]
    raw_profiles = settings.get("detector_profiles")
    if isinstance(raw_profiles, list):
        for raw_profile in raw_profiles:
            if isinstance(raw_profile, dict):
                profiles.append({**settings, **raw_profile})
    elif bool(settings.get("allow_detector_profile_fallback", True)):
        profiles.append(
            {
                **settings,
                "adaptive_thresh_window_max_px": 151,
                "adaptive_thresh_window_step_px": 8,
                "min_marker_perimeter_rate": 0.02,
                "min_corner_distance_rate": 0.005,
                "min_marker_distance_rate": 0.01,
                "error_correction_rate": 1.0,
            }
        )
    return profiles


def _order_quad_points(points: np.ndarray) -> np.ndarray:
    normalized = np.asarray(points, dtype=np.float32).reshape(4, 2)
    sums = normalized.sum(axis=1)
    differences = np.diff(normalized, axis=1).reshape(-1)
    return np.asarray(
        [
            normalized[np.argmin(sums)],
            normalized[np.argmin(differences)],
            normalized[np.argmax(sums)],
            normalized[np.argmax(differences)],
        ],
        dtype=np.float32,
    )


def _sample_marker_grid(image: np.ndarray, grid_size: int) -> np.ndarray:
    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _threshold, binary = cv2.threshold(
        gray,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU,
    )
    height, width = binary.shape[:2]
    bits = np.zeros((grid_size, grid_size), dtype=np.uint8)
    for row in range(grid_size):
        for column in range(grid_size):
            y0 = int((row + 0.25) * height / grid_size)
            y1 = int((row + 0.75) * height / grid_size)
            x0 = int((column + 0.25) * width / grid_size)
            x1 = int((column + 0.75) * width / grid_size)
            bits[row, column] = int(np.mean(binary[y0:y1, x0:x1]) > 127)
    return bits


def _expected_marker_hamming(
    warped_marker: np.ndarray,
    dictionary: cv2.aruco.Dictionary,
    marker_id: int,
    payload_only: bool = False,
) -> int:
    grid_size = int(dictionary.markerSize) + 2
    observed = _sample_marker_grid(warped_marker, grid_size)
    side_pixels = grid_size * 24
    reference = cv2.aruco.generateImageMarker(
        dictionary,
        int(marker_id),
        side_pixels,
        borderBits=1,
    )
    expected = _sample_marker_grid(reference, grid_size)
    if payload_only:
        observed = observed[1:-1, 1:-1]
        expected = expected[1:-1, 1:-1]
    return min(
        int(np.count_nonzero(observed != np.rot90(expected, rotation)))
        for rotation in range(4)
    )


def _marker_border_dark_fraction(warped_marker: np.ndarray, grid_size: int) -> float:
    observed = _sample_marker_grid(warped_marker, grid_size)
    border = np.concatenate(
        (
            observed[0, :],
            observed[-1, :],
            observed[1:-1, 0],
            observed[1:-1, -1],
        )
    )
    return float(np.mean(border == 0))


def _partial_reference_marker_candidate(
    gray: np.ndarray,
    dictionary: cv2.aruco.Dictionary,
    marker_id: int,
    reference_center: np.ndarray,
    marker_side_px: float,
    maximum_hamming: int,
    settings: dict[str, Any],
) -> np.ndarray | None:
    if not bool(settings.get("allow_partial_reference_markers", True)):
        return None
    grid_size = int(dictionary.markerSize) + 2
    canonical_size = grid_size * 28
    destination = np.asarray(
        [
            [0, 0],
            [canonical_size - 1, 0],
            [canonical_size - 1, canonical_size - 1],
            [0, canonical_size - 1],
        ],
        dtype=np.float32,
    )
    offset_range = float(settings.get("partial_marker_center_search_px", 5.0))
    angle_range = float(settings.get("partial_marker_angle_search_deg", 5.0))
    center_offsets = np.linspace(-offset_range, offset_range, 3)
    side_multipliers = (0.88, 0.96, 1.04, 1.12)
    angles = (-angle_range, 0.0, angle_range)
    minimum_contrast = float(settings.get("partial_marker_min_stddev", 20.0))
    minimum_dark_border = float(
        settings.get("partial_marker_min_dark_border_fraction", 0.65)
    )
    candidates: list[tuple[tuple[float, float, float], np.ndarray]] = []

    for offset_y in center_offsets:
        for offset_x in center_offsets:
            center = (
                float(reference_center[0] + offset_x),
                float(reference_center[1] + offset_y),
            )
            for side_multiplier in side_multipliers:
                side = max(12.0, marker_side_px * side_multiplier)
                for angle in angles:
                    ordered = _order_quad_points(
                        cv2.boxPoints((center, (side, side), float(angle)))
                    )
                    transform = cv2.getPerspectiveTransform(ordered, destination)
                    warped = cv2.warpPerspective(
                        gray,
                        transform,
                        (canonical_size, canonical_size),
                        flags=cv2.INTER_CUBIC,
                        borderMode=cv2.BORDER_REPLICATE,
                    )
                    contrast = float(np.std(warped))
                    if contrast < minimum_contrast:
                        continue
                    if (
                        _marker_border_dark_fraction(warped, grid_size)
                        < minimum_dark_border
                    ):
                        continue
                    hamming = _expected_marker_hamming(
                        warped,
                        dictionary,
                        marker_id,
                        payload_only=True,
                    )
                    if hamming > maximum_hamming:
                        continue
                    center_distance = abs(offset_x) + abs(offset_y)
                    side_distance = abs(1.0 - side_multiplier)
                    candidates.append(
                        (
                            (float(hamming), center_distance + side_distance, -contrast),
                            ordered,
                        )
                    )
    if not candidates:
        return None
    return min(candidates, key=lambda item: item[0])[1].astype(np.float32)


def _reference_guided_missing_markers(
    gray: np.ndarray,
    settings: dict[str, Any],
    dictionary_name: str,
    corners: list[np.ndarray],
    ids: np.ndarray | None,
) -> tuple[list[np.ndarray], np.ndarray | None, list[int]]:
    if not bool(settings.get("reference_guided_fallback", True)):
        return corners, ids, []
    references = scaled_reference_centers(settings, gray.shape)
    required_ids = [int(value) for value in settings.get("required_ids", [])]
    existing_ids = set(_visible_ids(ids))
    reference_ids = [
        marker_id
        for marker_id in required_ids
        if marker_id in references and marker_id not in existing_ids
    ]
    if not reference_ids:
        return corners, ids, []

    dictionary_id = getattr(cv2.aruco, str(dictionary_name).upper(), None)
    if dictionary_id is None:
        return corners, ids, []
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    scale = max(gray.shape[1] / 640.0, gray.shape[0] / 480.0)
    search_radius = max(
        40,
        int(round(float(settings.get("reference_search_radius_px", 82)) * scale)),
    )
    minimum_side = max(
        12.0,
        float(settings.get("reference_marker_min_side_px", 24.0)) * scale,
    )
    maximum_side = max(
        minimum_side + 1.0,
        float(settings.get("reference_marker_max_side_px", 80.0)) * scale,
    )
    maximum_hamming = max(
        0,
        int(settings.get("reference_decode_max_hamming", 1)),
    )
    canonical_size = (int(dictionary.markerSize) + 2) * 28
    destination = np.asarray(
        [
            [0, 0],
            [canonical_size - 1, 0],
            [canonical_size - 1, canonical_size - 1],
            [0, canonical_size - 1],
        ],
        dtype=np.float32,
    )
    recovered_corners: list[np.ndarray] = []
    recovered_ids: list[int] = []
    detected_side_lengths = [
        (
            float(np.linalg.norm(marker_corners[1] - marker_corners[0]))
            + float(np.linalg.norm(marker_corners[2] - marker_corners[1]))
        )
        / 2.0
        for marker_corners in corners
    ]
    typical_marker_side = (
        float(np.median(detected_side_lengths))
        if detected_side_lengths
        else float(settings.get("reference_marker_size_px", 44.0)) * scale
    )

    for marker_id in reference_ids:
        center = references[marker_id]
        x0 = max(0, int(round(center[0])) - search_radius)
        y0 = max(0, int(round(center[1])) - search_radius)
        x1 = min(gray.shape[1], int(round(center[0])) + search_radius)
        y1 = min(gray.shape[0], int(round(center[1])) + search_radius)
        if x1 - x0 < 16 or y1 - y0 < 16:
            continue
        roi = gray[y0:y1, x0:x1]
        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        _threshold, dark_mask = cv2.threshold(
            blurred,
            0,
            255,
            cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU,
        )
        contours, _hierarchy = cv2.findContours(
            dark_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        candidates: list[tuple[float, np.ndarray]] = []
        local_reference = center - np.asarray([x0, y0], dtype=np.float64)
        for contour in contours:
            rectangle = cv2.minAreaRect(contour)
            (candidate_x, candidate_y), (width, height), _angle = rectangle
            short_side = min(width, height)
            long_side = max(width, height)
            if (
                short_side < minimum_side
                or long_side > maximum_side
                or long_side / max(short_side, 1.0) > 1.65
            ):
                continue
            center_distance = float(
                np.linalg.norm(
                    np.asarray([candidate_x, candidate_y]) - local_reference
                )
            )
            if center_distance > search_radius * 0.75:
                continue
            ordered = _order_quad_points(cv2.boxPoints(rectangle))
            transform = cv2.getPerspectiveTransform(ordered, destination)
            warped = cv2.warpPerspective(
                roi,
                transform,
                (canonical_size, canonical_size),
                flags=cv2.INTER_CUBIC,
                borderMode=cv2.BORDER_REPLICATE,
            )
            hamming = _expected_marker_hamming(warped, dictionary, marker_id)
            if hamming <= maximum_hamming:
                candidates.append((center_distance + hamming * 1000.0, ordered))
        if candidates:
            _score, selected = min(candidates, key=lambda item: item[0])
            selected[:, 0] += x0
            selected[:, 1] += y0
        else:
            patch_half_side = max(8, int(round(typical_marker_side * 0.65)))
            patch_x0 = max(0, int(round(center[0])) - patch_half_side)
            patch_y0 = max(0, int(round(center[1])) - patch_half_side)
            patch_x1 = min(gray.shape[1], int(round(center[0])) + patch_half_side)
            patch_y1 = min(gray.shape[0], int(round(center[1])) + patch_half_side)
            center_patch = gray[patch_y0:patch_y1, patch_x0:patch_x1]
            if center_patch.size == 0:
                continue
            dark_threshold = min(
                120.0,
                float(np.median(center_patch)) * 0.72,
            )
            dark_fraction = float(np.mean(center_patch < dark_threshold))
            if dark_fraction < float(
                settings.get("partial_marker_min_dark_fraction", 0.12)
            ):
                continue
            selected = _partial_reference_marker_candidate(
                gray,
                dictionary,
                marker_id,
                center,
                typical_marker_side,
                maximum_hamming,
                settings,
            )
            if selected is None:
                continue
        recovered_corners.append(selected.astype(np.float32))
        recovered_ids.append(marker_id)

    if not recovered_ids:
        return corners, ids, []
    existing_order = (
        [] if ids is None else [int(marker_id) for marker_id in ids.reshape(-1)]
    )
    recovered_centers = [
        np.mean(marker_corners, axis=0) for marker_corners in recovered_corners
    ]
    kept_corners: list[np.ndarray] = []
    kept_ids: list[int] = []
    for marker_id, marker_corners in zip(existing_order, corners, strict=True):
        center = np.mean(marker_corners, axis=0)
        if any(
            float(np.linalg.norm(center - recovered_center)) < minimum_side
            for recovered_center in recovered_centers
        ):
            continue
        kept_corners.append(marker_corners)
        kept_ids.append(marker_id)
    merged_corners = [*kept_corners, *recovered_corners]
    merged_ids = [*kept_ids, *recovered_ids]
    return (
        merged_corners,
        np.asarray(merged_ids, dtype=np.int32).reshape(-1, 1),
        recovered_ids,
    )


def _mode_images(gray: np.ndarray, settings: dict[str, Any]) -> list[tuple[str, np.ndarray]]:
    invert_first = bool(settings.get("invert_first", False))
    allow_fallback = bool(settings.get("allow_normal_fallback", True))
    first = ("inverted", 255 - gray) if invert_first else ("normal", gray)
    second = ("normal", gray) if invert_first else ("inverted", 255 - gray)
    return [first, second] if allow_fallback else [first]


def _candidate_mode_images(
    gray: np.ndarray,
    settings: dict[str, Any],
) -> list[tuple[str, np.ndarray, bool]]:
    modes = _mode_images(gray, settings)
    candidates = [(mode, source, False) for mode, source in modes]
    if bool(settings.get("allow_mirror_fallback", True)):
        candidates.extend(
            (f"{mode}+mirror_x", cv2.flip(source, 1), True)
            for mode, source in modes
        )
    return candidates


def _restore_mirrored_corners(corners: np.ndarray, image_width: int) -> np.ndarray:
    restored = np.asarray(corners, dtype=np.float32).reshape(4, 2).copy()
    restored[:, 0] = float(image_width - 1) - restored[:, 0]
    return _order_quad_points(restored)


def _score_detection(
    ids: np.ndarray | None,
    rejected_count: int,
    required_ids: set[int],
    dictionary_index: int,
    mode_index: int,
) -> tuple[int, int, int, int, int]:
    visible = set(_visible_ids(ids))
    required_seen = len(required_ids.intersection(visible)) if required_ids else len(visible)
    return (
        required_seen,
        len(required_ids.intersection(visible)) if required_ids else len(visible),
        -dictionary_index,
        -mode_index,
        -int(rejected_count),
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
    modes = _candidate_mode_images(gray, settings)
    required_ids = _as_int_set(settings.get("required_ids"))
    best: FiducialDetection | None = None
    best_score = (-1, -1, -10_000, -10_000, -10_000)
    last_error: ValueError | None = None
    valid_dictionary_seen = False

    for dictionary_index, dictionary_name in enumerate(dictionaries):
        detectors: list[cv2.aruco.ArucoDetector] = []
        for profile in _detector_setting_profiles(settings):
            try:
                detectors.append(make_aruco_detector(dictionary_name, profile))
            except ValueError as exc:
                last_error = exc
                detectors = []
                break
        if not detectors:
            continue
        valid_dictionary_seen = True
        for mode_index, (mode, source, mirrored) in enumerate(modes):
            merged: dict[int, np.ndarray] = {}
            rejected_count = 0
            for detector in detectors:
                corners, ids, rejected = detector.detectMarkers(source)
                rejected_count += len(rejected)
                for marker_corners, marker_id in zip(
                    corners,
                    [] if ids is None else ids.reshape(-1),
                    strict=True,
                ):
                    normalized_id = int(marker_id)
                    if normalized_id in merged:
                        continue
                    if required_ids and normalized_id not in required_ids:
                        continue
                    marker_points = np.asarray(
                        marker_corners,
                        dtype=np.float32,
                    ).reshape(4, 2)
                    if mirrored:
                        marker_points = _restore_mirrored_corners(
                            marker_points,
                            gray.shape[1],
                        )
                    merged[normalized_id] = marker_points
            normalized_ids = sorted(merged)
            normalized = [merged[marker_id] for marker_id in normalized_ids]
            filtered_ids = (
                np.asarray(normalized_ids, dtype=np.int32).reshape(-1, 1)
                if normalized_ids
                else None
            )
            score = _score_detection(
                filtered_ids,
                rejected_count,
                required_ids,
                dictionary_index,
                mode_index,
            )
            if score > best_score:
                best_score = score
                best = FiducialDetection(
                    corners=normalized,
                    ids=filtered_ids,
                    dictionary=dictionary_name,
                    mode=mode if normalized_ids else "none",
                    visible_ids=sorted(normalized_ids),
                    rejected_count=rejected_count,
                )

    if not valid_dictionary_seen and last_error is not None:
        raise last_error
    if best is None:
        return FiducialDetection([], None, dictionaries[0], "none", [], 0)
    if (
        required_ids
        and not required_ids.intersection(best.visible_ids)
        and best.dictionary != dictionaries[0]
    ):
        best = FiducialDetection(
            corners=[],
            ids=None,
            dictionary=dictionaries[0],
            mode="none",
            visible_ids=[],
            rejected_count=best.rejected_count,
        )
    recovered_corners, recovered_ids, guided_ids = _reference_guided_missing_markers(
        gray,
        settings,
        best.dictionary,
        best.corners,
        best.ids,
    )
    visible_after_first_pass = set(_visible_ids(recovered_ids))
    if required_ids - visible_after_first_pass:
        smoothed = cv2.GaussianBlur(gray, (3, 3), 0)
        recovered_corners, recovered_ids, smoothed_ids = (
            _reference_guided_missing_markers(
                smoothed,
                settings,
                best.dictionary,
                recovered_corners,
                recovered_ids,
            )
        )
        guided_ids.extend(smoothed_ids)
    if not guided_ids:
        return best
    return FiducialDetection(
        corners=recovered_corners,
        ids=recovered_ids,
        dictionary=best.dictionary,
        mode=f"{best.mode}+reference_guided",
        visible_ids=_visible_ids(recovered_ids),
        rejected_count=best.rejected_count,
    )


def marker_centers(corners: list[np.ndarray], ids: np.ndarray | None) -> dict[int, np.ndarray]:
    centers: dict[int, np.ndarray] = {}
    if ids is None:
        return centers
    for marker_corners, marker_id in zip(corners, ids.reshape(-1), strict=True):
        centers[int(marker_id)] = np.mean(marker_corners.reshape(4, 2), axis=0).astype(np.float32)
    return centers


def marker_box_corners(
    corners: list[np.ndarray],
    ids: np.ndarray | None,
    tag_box_corner_index: dict[str, Any] | dict[int, Any],
) -> dict[int, np.ndarray]:
    selected: dict[int, np.ndarray] = {}
    if ids is None:
        return selected
    detected_centers = marker_centers(corners, ids)
    workspace_center = (
        np.mean(np.asarray(list(detected_centers.values())), axis=0)
        if len(detected_centers) >= 3
        else None
    )
    for marker_corners, marker_id in zip(corners, ids.reshape(-1), strict=True):
        normalized_id = int(marker_id)
        normalized_corners = marker_corners.reshape(4, 2)
        if workspace_center is not None:
            corner_index = int(
                np.argmax(np.linalg.norm(normalized_corners - workspace_center, axis=1))
            )
            selected[normalized_id] = normalized_corners[corner_index].astype(
                np.float32
            )
            continue
        raw_index = tag_box_corner_index.get(
            str(normalized_id),
            tag_box_corner_index.get(normalized_id),
        )
        if raw_index is None:
            continue
        try:
            corner_index = int(raw_index)
        except (TypeError, ValueError):
            continue
        if 0 <= corner_index <= 3:
            selected[normalized_id] = normalized_corners[corner_index].astype(np.float32)
    return selected


def marker_box_polygon(
    corners: list[np.ndarray],
    ids: np.ndarray | None,
    tag_box_corner_index: dict[str, Any] | dict[int, Any],
) -> np.ndarray | None:
    if ids is None or not corners:
        return None
    selected = marker_box_corners(corners, ids, tag_box_corner_index)
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


def workspace_robot_corners_by_tag(settings: dict[str, Any]) -> dict[int, np.ndarray]:
    required_ids = [int(value) for value in settings.get("required_ids", [])]
    tag_centers = settings.get("tag_centers_robot_mm")
    polygon_points = workspace_polygon_robot_points(settings)
    if not required_ids or not isinstance(tag_centers, dict) or polygon_points is None:
        return {}
    if len(polygon_points) < len(required_ids) or not np.all(np.isfinite(polygon_points)):
        return {}

    result: dict[int, np.ndarray] = {}
    used_polygon_indices: set[int] = set()
    for marker_id in required_ids:
        raw_center = tag_centers.get(str(marker_id), tag_centers.get(marker_id))
        if not isinstance(raw_center, (list, tuple)) or len(raw_center) < 2:
            return {}
        try:
            center = np.asarray([float(raw_center[0]), float(raw_center[1])], dtype=np.float64)
        except (TypeError, ValueError):
            return {}
        distances = np.linalg.norm(polygon_points - center, axis=1)
        polygon_index = int(np.argmin(distances))
        if polygon_index in used_polygon_indices:
            return {}
        used_polygon_indices.add(polygon_index)
        result[marker_id] = polygon_points[polygon_index]
    return result


def workspace_margin_mm(settings: dict[str, Any]) -> float:
    try:
        margin = float(settings.get("workspace_margin_mm", 0.0))
    except (TypeError, ValueError):
        return 0.0
    if not np.isfinite(margin):
        return 0.0
    return max(0.0, margin)


def workspace_polygon_robot_points(
    settings: dict[str, Any],
    *,
    include_margin: bool = False,
) -> np.ndarray | None:
    polygon = settings.get("workspace_polygon_robot_mm")
    if not isinstance(polygon, list) or len(polygon) < 3:
        return None
    try:
        points = np.asarray(
            [[float(point[0]), float(point[1])] for point in polygon],
            dtype=np.float64,
        )
    except (TypeError, ValueError, IndexError):
        return None
    if not np.all(np.isfinite(points)):
        return None

    margin = workspace_margin_mm(settings) if include_margin else 0.0
    if margin <= 0:
        return points

    min_x = float(np.min(points[:, 0])) - margin
    max_x = float(np.max(points[:, 0])) + margin
    min_y = float(np.min(points[:, 1])) - margin
    max_y = float(np.max(points[:, 1])) + margin
    return np.asarray(
        [
            [min_x, min_y],
            [max_x, min_y],
            [max_x, max_y],
            [min_x, max_y],
        ],
        dtype=np.float64,
    )


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
    workspace_corners_px: dict[int, np.ndarray] | None = None,
) -> tuple[np.ndarray | None, dict[str, Any]]:
    required_ids = [int(value) for value in settings.get("required_ids", [])]
    robot_positions = settings.get("tag_centers_robot_mm")
    if not isinstance(robot_positions, dict) or not required_ids:
        return None, {"ok": False, "error": "workspace tag robot positions are not configured"}
    if any(marker_id not in centers for marker_id in required_ids):
        missing = [marker_id for marker_id in required_ids if marker_id not in centers]
        return None, {"ok": False, "error": f"missing required workspace tags: {missing}"}
    robot_points_array = _point_list_from_mapping(robot_positions, required_ids)
    if robot_points_array is None:
        return None, {"ok": False, "error": "workspace tag robot positions are invalid"}

    configured_corners = workspace_robot_corners_by_tag(settings)
    corner_ids = (
        [
            marker_id
            for marker_id in required_ids
            if marker_id in workspace_corners_px and marker_id in configured_corners
        ]
        if workspace_corners_px and configured_corners
        else []
    )
    use_outer_corners = len(corner_ids) == len(required_ids) and len(corner_ids) >= 4
    if use_outer_corners:
        image_points = [
            np.asarray(workspace_corners_px[marker_id], dtype=np.float64)
            for marker_id in required_ids
        ]
        robot_points = [configured_corners[marker_id] for marker_id in required_ids]
        fit_source = "workspace_outer_corners"
    else:
        image_points = [
            np.asarray(centers[marker_id], dtype=np.float64)
            for marker_id in required_ids
        ]
        robot_points = [point for point in robot_points_array]
        fit_source = "tag_centers"

    homography, metrics = solve_image_to_robot_homography(
        np.asarray(image_points, dtype=np.float64),
        np.asarray(robot_points, dtype=np.float64),
    )
    return homography, {
        **metrics,
        "tags_used": required_ids,
        "fit_source": fit_source,
        "tag_center_point_count": 0 if use_outer_corners else len(required_ids),
        "workspace_corner_point_count": len(required_ids) if use_outer_corners else 0,
        "observed_tag_center_count": len(required_ids),
    }


def scaled_reference_points(
    settings: dict[str, Any],
    image_shape: tuple[int, ...],
    key: str,
) -> dict[int, np.ndarray]:
    raw_points = settings.get(key)
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


def scaled_reference_centers(
    settings: dict[str, Any],
    image_shape: tuple[int, ...],
) -> dict[int, np.ndarray]:
    return scaled_reference_points(settings, image_shape, "reference_points_px")


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
    reference_workspace_corners = scaled_reference_points(
        settings,
        image_shape,
        "reference_workspace_corners_px",
    )
    homography, metrics = homography_from_marker_centers(
        references,
        settings,
        reference_workspace_corners,
    )
    if homography is None:
        return None, metrics
    if _point_list_from_mapping(robot_positions, required_ids) is None:
        return None, {"ok": False, "error": "saved workspace robot points are invalid"}
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
    max_scale_deviation = float(
        settings.get("max_layout_pair_scale_deviation", 0.35)
    )
    mismatches: list[str] = []
    errors: dict[int, float] = {}
    pair_scales: list[tuple[int, int, float]] = []
    for first_index, first_id in enumerate(visible_required):
        for second_id in visible_required[first_index + 1 :]:
            live_delta = centers[second_id] - centers[first_id]
            reference_delta = references[second_id] - references[first_id]
            live_norm = float(np.linalg.norm(live_delta))
            reference_norm = float(np.linalg.norm(reference_delta))
            if live_norm < 1e-6 or reference_norm < 1e-6:
                continue
            alignment = float(np.dot(live_delta, reference_delta) / (live_norm * reference_norm))
            pair_scales.append(
                (first_id, second_id, live_norm / reference_norm)
            )
            if alignment < min_alignment:
                mismatches.append(
                    f"tag vector {first_id}->{second_id} points opposite the saved workspace layout"
                )
                errors[first_id] = max(errors.get(first_id, 0.0), abs(alignment))
                errors[second_id] = max(errors.get(second_id, 0.0), abs(alignment))
    if pair_scales:
        median_scale = float(np.median([scale for _first, _second, scale in pair_scales]))
        if median_scale > 1e-6:
            for first_id, second_id, scale in pair_scales:
                deviation = abs(scale / median_scale - 1.0)
                if deviation > max_scale_deviation:
                    mismatches.append(
                        f"tag spacing {first_id}<->{second_id} does not match the saved workspace layout"
                    )
                    errors[first_id] = max(errors.get(first_id, 0.0), deviation)
                    errors[second_id] = max(errors.get(second_id, 0.0), deviation)
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
    robot_points = workspace_polygon_robot_points(settings, include_margin=True)
    if robot_points is None:
        return None
    robot_to_image = np.linalg.inv(homography_image_to_robot)
    image_points = cv2.perspectiveTransform(robot_points.reshape(-1, 1, 2), robot_to_image).reshape(-1, 2)
    return image_points.astype(np.float32)


def workspace_mapping_errors(
    homography: np.ndarray,
    detection: FiducialDetection,
    settings: dict[str, Any],
) -> dict[str, Any]:
    required_ids = [int(value) for value in settings.get("required_ids", [])]
    centers = marker_centers(detection.corners, detection.ids)
    corners = marker_box_corners(
        detection.corners,
        detection.ids,
        settings.get("tag_box_corner_index", {}),
    )
    expected_centers = settings.get("tag_centers_robot_mm")
    expected_corners = workspace_robot_corners_by_tag(settings)
    if not isinstance(expected_centers, dict):
        return {"ok": False, "error": "workspace tag robot positions are not configured"}

    center_errors: list[float] = []
    corner_errors: list[float] = []
    per_tag: dict[str, dict[str, Any]] = {}
    for marker_id in required_ids:
        tag_errors: dict[str, float] = {}
        expected_center = expected_centers.get(str(marker_id), expected_centers.get(marker_id))
        if marker_id in centers and isinstance(expected_center, (list, tuple)):
            mapped = apply_homography(homography, centers[marker_id])
            error = float(
                np.linalg.norm(
                    np.asarray([mapped["x_mm"], mapped["y_mm"]], dtype=np.float64)
                    - np.asarray(expected_center[:2], dtype=np.float64)
                )
            )
            tag_errors["center_error_mm"] = error
            center_errors.append(error)
        if marker_id in corners and marker_id in expected_corners:
            mapped = apply_homography(homography, corners[marker_id])
            error = float(
                np.linalg.norm(
                    np.asarray([mapped["x_mm"], mapped["y_mm"]], dtype=np.float64)
                    - expected_corners[marker_id]
                )
            )
            tag_errors["workspace_corner_error_mm"] = error
            corner_errors.append(error)
        if tag_errors:
            per_tag[str(marker_id)] = tag_errors

    missing_ids = [marker_id for marker_id in required_ids if marker_id not in centers]
    acceptance_errors = corner_errors or center_errors
    if not acceptance_errors:
        return {
            "ok": False,
            "error": "no configured workspace tags were detected",
            "missing_ids": missing_ids,
            "per_tag": per_tag,
        }
    rmse = float(np.sqrt(np.mean(np.asarray(acceptance_errors) ** 2)))
    maximum = float(np.max(acceptance_errors))
    accepted = bool(
        not missing_ids
        and rmse <= float(settings.get("max_verification_rmse_mm", 5.0))
        and maximum <= float(settings.get("max_verification_error_mm", 10.0))
    )
    return {
        "ok": accepted,
        "accepted": accepted,
        "point_count": len(acceptance_errors),
        "corner_point_count": len(corner_errors),
        "center_point_count": len(center_errors),
        "rmse_mm": rmse,
        "max_error_mm": maximum,
        "missing_ids": missing_ids,
        "per_tag": per_tag,
    }


def annotate_workspace_calibration(
    image_bgr: np.ndarray,
    detection: FiducialDetection,
    settings: dict[str, Any],
    result: dict[str, Any] | None = None,
) -> np.ndarray:
    annotated = image_bgr.copy()
    if detection.ids is not None and detection.corners:
        cv2.aruco.drawDetectedMarkers(
            annotated,
            [corner.astype(np.float32).reshape(1, 4, 2) for corner in detection.corners],
            detection.ids,
        )
    centers = marker_centers(detection.corners, detection.ids)
    outer_corners = marker_box_corners(
        detection.corners,
        detection.ids,
        settings.get("tag_box_corner_index", {}),
    )
    robot_centers = settings.get("tag_centers_robot_mm")
    if not isinstance(robot_centers, dict):
        robot_centers = {}
    for marker_id, center in centers.items():
        point = tuple(np.round(center).astype(int))
        expected = robot_centers.get(str(marker_id), robot_centers.get(marker_id))
        label = f"ID {marker_id}"
        if isinstance(expected, (list, tuple)) and len(expected) >= 2:
            label += f" -> X {float(expected[0]):.1f}, Y {float(expected[1]):.1f} mm"
        cv2.putText(
            annotated,
            label,
            (max(4, point[0] + 7), max(18, point[1] - 7)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (70, 245, 180),
            1,
            cv2.LINE_AA,
        )
    for point in outer_corners.values():
        cv2.circle(annotated, tuple(np.round(point).astype(int)), 6, (0, 220, 255), -1)
    polygon = marker_box_polygon(
        detection.corners,
        detection.ids,
        settings.get("tag_box_corner_index", {}),
    )
    if polygon is not None:
        cv2.polylines(
            annotated,
            [np.round(polygon).astype(np.int32)],
            True,
            (0, 220, 255),
            3,
            cv2.LINE_AA,
        )

    status = "Workspace tags: " + (", ".join(map(str, detection.visible_ids)) or "none")
    if result:
        metrics = result.get("metrics") if isinstance(result.get("metrics"), dict) else result
        if result.get("ready"):
            status += " | ready to save"
        elif metrics.get("rmse_mm") is not None:
            status += f" | {float(metrics['rmse_mm']):.2f} mm RMSE"
        elif result.get("error"):
            status += " | more samples required"
    cv2.rectangle(
        annotated,
        (8, 8),
        (min(annotated.shape[1] - 8, 690), 40),
        (12, 16, 24),
        -1,
    )
    cv2.putText(
        annotated,
        status,
        (16, 31),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.56,
        (235, 240, 248),
        2,
        cv2.LINE_AA,
    )
    return annotated


def serialize_homography(homography: np.ndarray) -> list[list[float]]:
    matrix = np.asarray(homography, dtype=np.float64)
    if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
        raise ValueError("homography must be a finite 3x3 matrix")
    return [[float(value) for value in row] for row in matrix]
