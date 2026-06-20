from __future__ import annotations

import base64
from dataclasses import dataclass
from threading import RLock
from time import sleep, time
from typing import Any
from uuid import uuid4

import cv2
import numpy as np

from .apriltag_calibration import project_image_point_to_plane
from .workspace_calibration import (
    FiducialDetection,
    apply_homography,
    dictionary_candidates,
    polygon_from_robot,
    saved_homography,
    solve_image_to_robot_homography,
)


def planar_transform_from_points(
    image_points: list[list[float]],
    robot_points: list[list[float]],
) -> np.ndarray:
    if len(image_points) != 4 or len(robot_points) != 4:
        raise ValueError("planar calibration requires exactly four image and robot points")
    transform, metrics = solve_image_to_robot_homography(
        np.asarray(image_points, dtype=np.float64),
        np.asarray(robot_points, dtype=np.float64),
    )
    if transform is None:
        raise ValueError(str(metrics.get("error", "could not solve planar transform")))
    return transform


def apply_planar_transform(transform: np.ndarray, image_point: list[float]) -> dict[str, float]:
    return apply_homography(transform, image_point)


def decode_image_b64(image_b64: str) -> np.ndarray:
    payload = image_b64.split(",", 1)[-1]
    raw = base64.b64decode(payload)
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("could not decode image")
    return image


def encode_image_b64(
    image_bgr: np.ndarray,
    ext: str = ".jpg",
    parameters: list[int] | None = None,
) -> str:
    ok, encoded = cv2.imencode(ext, image_bgr, parameters or [])
    if not ok:
        raise ValueError("could not encode image")
    mime = "image/png" if ext.lower() == ".png" else "image/jpeg"
    payload = base64.b64encode(encoded.tobytes()).decode("ascii")
    return f"data:{mime};base64,{payload}"


class CameraCapture:
    """Thread-safe persistent OpenCV camera handle shared by vision endpoints."""

    def __init__(self) -> None:
        self._lock = RLock()
        self._capture: cv2.VideoCapture | None = None
        self._signature: tuple[int, int, int] | None = None

    @staticmethod
    def _camera_signature(camera: dict[str, Any]) -> tuple[int, int, int]:
        resolution = camera.get("resolution") if isinstance(camera.get("resolution"), dict) else {}
        return (
            int(camera.get("source_index", 0)),
            int(resolution.get("width", 0) or 0),
            int(resolution.get("height", 0) or 0),
        )

    def _ensure_open(self, camera: dict[str, Any]) -> cv2.VideoCapture:
        signature = self._camera_signature(camera)
        if self._capture is not None and self._signature == signature and self._capture.isOpened():
            return self._capture
        self.close()
        source_index, width, height = signature
        capture = cv2.VideoCapture(source_index)
        if width > 0:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        if height > 0:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"could not open camera index {source_index}")
        self._capture = capture
        self._signature = signature
        return capture

    @staticmethod
    def _orient_image(image: np.ndarray, camera: dict[str, Any]) -> np.ndarray:
        display = camera.get("display") if isinstance(camera.get("display"), dict) else {}
        return cv2.flip(image, 1) if bool(display.get("flip_x", False)) else image

    def read(self, camera: dict[str, Any]) -> np.ndarray:
        with self._lock:
            capture = self._ensure_open(camera)
            ok, image = capture.read()
            if not ok or image is None:
                self.close()
                raise RuntimeError("could not read camera frame")
            return self._orient_image(image, camera)

    def read_many(
        self,
        camera: dict[str, Any],
        sample_count: int,
        interval_s: float = 0.0,
    ) -> list[np.ndarray]:
        images: list[np.ndarray] = []
        with self._lock:
            capture = self._ensure_open(camera)
            for index in range(max(1, sample_count)):
                ok, image = capture.read()
                if not ok or image is None:
                    self.close()
                    raise RuntimeError("could not read camera frame")
                images.append(self._orient_image(image, camera))
                if index + 1 < sample_count and interval_s > 0:
                    sleep(interval_s)
        return images

    def close(self) -> None:
        with self._lock:
            if self._capture is not None:
                self._capture.release()
            self._capture = None
            self._signature = None


def detect_color_blob(image_bgr: np.ndarray, profile: dict[str, Any]) -> dict[str, Any]:
    """Compatibility helper for one configured HSV profile."""

    hsv_min = np.array(profile.get("hsv_min", [0, 0, 0]), dtype=np.uint8)
    hsv_max = np.array(profile.get("hsv_max", [179, 255, 255]), dtype=np.uint8)
    min_area = float(profile.get("min_area_px", 200.0))
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_min, hsv_max)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return {"ok": False, "reason": "no contour", "area_px": 0.0}
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    if area < min_area:
        return {"ok": False, "reason": "below min area", "area_px": area}
    moments = cv2.moments(contour)
    if abs(moments["m00"]) < 1e-9:
        return {"ok": False, "reason": "empty contour moment", "area_px": area}
    cx = float(moments["m10"] / moments["m00"])
    cy = float(moments["m01"] / moments["m00"])
    x, y, width, height = cv2.boundingRect(contour)
    return {
        "ok": True,
        "center_px": {"x": cx, "y": cy},
        "area_px": area,
        "bbox_px": {"x": int(x), "y": int(y), "w": int(width), "h": int(height)},
    }


def _matrix_from_saved_planar(saved_result: dict[str, Any]) -> np.ndarray | None:
    planar = saved_result.get("planar") if isinstance(saved_result.get("planar"), dict) else {}
    raw = planar.get("homography_image_to_robot")
    if raw is None:
        return None
    matrix = np.asarray(raw, dtype=np.float64)
    if matrix.shape == (3, 3) and np.all(np.isfinite(matrix)):
        return matrix
    return None


def _projection_from_calibration(calibration: dict[str, Any] | None) -> dict[str, Any]:
    calibration = calibration or {}
    april_tag = calibration.get("apriltag") if isinstance(calibration.get("apriltag"), dict) else {}
    saved_result = april_tag.get("result") if isinstance(april_tag.get("result"), dict) else {}
    if saved_result.get("accepted"):
        return {
            "source": "apriltag_camera_pose",
            "apriltag_result": saved_result,
            "pose_id": saved_result.get("id"),
            "pose_timestamp": saved_result.get("timestamp"),
        }
    saved_planar = _matrix_from_saved_planar(saved_result)
    if saved_planar is not None:
        return {
            "source": "apriltag_planar_homography",
            "homography": saved_planar,
            "pose_id": saved_result.get("id"),
            "pose_timestamp": saved_result.get("timestamp"),
        }

    image_points = calibration.get("image_points") or []
    robot_points = calibration.get("robot_points") or []
    if len(image_points) == 4 and len(robot_points) == 4:
        return {
            "source": "planar_homography",
            "homography": planar_transform_from_points(image_points, robot_points),
        }
    return {"source": "unavailable"}


def _project_center(
    center_px: tuple[float, float],
    projection: dict[str, Any],
) -> tuple[dict[str, float] | None, str | None]:
    source = str(projection.get("source", "unavailable"))
    try:
        if source == "apriltag_camera_pose":
            return (
                project_image_point_to_plane(
                    [center_px[0], center_px[1]],
                    projection["apriltag_result"],
                ),
                None,
            )
        if source in {
            "workspace_aruco_saved",
            "planar_homography",
            "apriltag_planar_homography",
        }:
            return apply_homography(projection["homography"], center_px), None
    except (KeyError, ValueError, cv2.error, np.linalg.LinAlgError) as exc:
        return None, str(exc)
    return None, "no valid camera-to-robot calibration"


def _detection_contract(
    *,
    label: str,
    center_px: tuple[float, float],
    bbox_px: tuple[int, int, int, int],
    area_px: float,
    projection: dict[str, Any],
    confidence: float,
    detector: str,
    drop_zone: str | None = None,
    task_eligible: bool = False,
    detection_id: str | None = None,
) -> dict[str, Any]:
    x, y, width, height = bbox_px
    robot, projection_error = _project_center(center_px, projection)
    result: dict[str, Any] = {
        "id": detection_id or str(uuid4()),
        "ok": True,
        "label": label,
        "color": label,
        "confidence": float(max(0.0, min(1.0, confidence))),
        "quality": float(max(0.0, min(1.0, confidence))),
        "detector": detector,
        "timestamp": time(),
        "center_px": {"x": float(center_px[0]), "y": float(center_px[1])},
        "image": {
            "x_px": float(center_px[0]),
            "y_px": float(center_px[1]),
            "bbox_px": {"x": int(x), "y": int(y), "w": int(width), "h": int(height)},
        },
        "bbox_px": {"x": int(x), "y": int(y), "w": int(width), "h": int(height)},
        "area_px": float(area_px),
        "robot": robot,
        "coordinate_source": projection.get("source", "unavailable"),
        "drop_zone": drop_zone,
        "task_eligible": bool(task_eligible and robot is not None),
    }
    if projection.get("pose_id"):
        result["camera_pose_id"] = projection.get("pose_id")
        result["camera_pose_timestamp"] = projection.get("pose_timestamp")
        result["projection_quality"] = {
            "confidence": (projection.get("apriltag_result", {}).get("metrics") or {}).get("confidence"),
            "reprojection_rmse_px": (
                projection.get("apriltag_result", {}).get("metrics") or {}
            ).get("reprojection_rmse_px"),
        }
    if projection_error:
        result["projection_error"] = projection_error
    return result


def detect_configured_colors(
    image_bgr: np.ndarray,
    profiles: dict[str, dict[str, Any]],
    calibration: dict[str, Any] | None = None,
    projection: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    projection = projection or _projection_from_calibration(calibration)
    detections: list[dict[str, Any]] = []
    for name, profile in profiles.items():
        if not bool(profile.get("enabled", True)):
            continue
        result = detect_color_blob(image_bgr, profile)
        if not result.get("ok"):
            detections.append(
                {
                    **result,
                    "id": f"{name}-none",
                    "label": name,
                    "color": name,
                    "confidence": 0.0,
                    "quality": 0.0,
                    "detector": "configured_hsv_blob",
                    "timestamp": time(),
                    "drop_zone": profile.get("drop_zone"),
                    "task_eligible": False,
                }
            )
            continue
        center = result["center_px"]
        bbox = result["bbox_px"]
        minimum = max(float(profile.get("min_area_px", 200.0)), 1.0)
        detections.append(
            _detection_contract(
                label=name,
                center_px=(float(center["x"]), float(center["y"])),
                bbox_px=(int(bbox["x"]), int(bbox["y"]), int(bbox["w"]), int(bbox["h"])),
                area_px=float(result["area_px"]),
                projection=projection,
                confidence=min(1.0, float(result["area_px"]) / (minimum * 4.0)),
                detector="configured_hsv_blob",
                drop_zone=profile.get("drop_zone"),
                task_eligible=True,
                detection_id=f"{name}-1",
            )
        )
    return detections


def workspace_aruco_settings(camera: dict[str, Any]) -> dict[str, Any]:
    calibration = camera.get("calibration") if isinstance(camera.get("calibration"), dict) else {}
    raw = (
        calibration.get("workspace_aruco")
        if isinstance(calibration.get("workspace_aruco"), dict)
        else {}
    )
    defaults: dict[str, Any] = {
        "enabled": False,
        "dictionary": "DICT_4X4_50",
        "dictionary_candidates": ["DICT_4X4_50", "DICT_APRILTAG_36H11"],
        "required_ids": [0, 1, 2, 3],
        "invert_first": True,
        "allow_normal_fallback": True,
        "tag_centers_robot_mm": {},
        "tag_box_corner_index": {"0": 0, "1": 1, "2": 3, "3": 2},
        "reference_points_px": {},
        "reference_workspace_corners_px": {},
        "reference_resolution": {"width": 640, "height": 480},
        "workspace_polygon_robot_mm": [],
        "projection_polygon_robot_mm": [],
        "projection_mode": "workplate",
        "projection_padding_mm": 0.0,
        "projection_pixels_per_mm": 1.0,
        "projection_jpeg_quality": 82,
        "projection_alpha": 220,
        "min_layout_vector_alignment": 0.2,
        "minimum_samples": 12,
        "max_calibration_rmse_mm": 3.0,
        "max_calibration_error_mm": 7.0,
        "max_calibration_tag_center_error_mm": 12.0,
        "max_verification_rmse_mm": 5.0,
        "max_verification_error_mm": 10.0,
    }
    defaults.update(raw)
    return defaults


def _classify_color(hsv_pixels: np.ndarray) -> str:
    if hsv_pixels.size == 0:
        return "unknown"
    hue = float(np.median(hsv_pixels[:, 0]))
    saturation = float(np.median(hsv_pixels[:, 1]))
    value = float(np.median(hsv_pixels[:, 2]))
    if value < 50:
        return "black"
    if saturation < 45:
        return "white" if value > 190 else "gray"
    if hue < 10 or hue >= 170:
        return "red"
    if hue < 25:
        return "orange"
    if hue < 35:
        return "yellow"
    if hue < 85:
        return "green"
    if hue < 100:
        return "cyan"
    if hue < 130:
        return "blue"
    if hue < 155:
        return "purple"
    return "pink"


def _workspace_mask(shape: tuple[int, int], polygon_px: np.ndarray | None) -> np.ndarray | None:
    if polygon_px is None or len(polygon_px) < 3:
        return None
    mask = np.zeros(shape, dtype=np.uint8)
    cv2.fillPoly(mask, [np.round(polygon_px).astype(np.int32)], 255)
    return mask


def _workspace_robot_polygon(settings: dict[str, Any]) -> np.ndarray | None:
    polygon = settings.get("projection_polygon_robot_mm")
    if not isinstance(polygon, list) or len(polygon) < 3:
        polygon = settings.get("workspace_polygon_robot_mm")
    if not isinstance(polygon, list) or len(polygon) < 3:
        return None
    try:
        points = np.asarray([[float(point[0]), float(point[1])] for point in polygon], dtype=np.float64)
    except (TypeError, ValueError, IndexError):
        return None
    if not np.all(np.isfinite(points)):
        return None
    padding = max(0.0, float(settings.get("projection_padding_mm", 0.0)))
    if padding > 0:
        min_x = float(np.min(points[:, 0])) - padding
        max_x = float(np.max(points[:, 0])) + padding
        min_y = float(np.min(points[:, 1])) - padding
        max_y = float(np.max(points[:, 1])) + padding
        points = np.asarray(
            [
                [min_x, min_y],
                [max_x, min_y],
                [max_x, max_y],
                [min_x, max_y],
            ],
            dtype=np.float64,
        )
    return points


def _workspace_projection_texture(
    image_bgr: np.ndarray,
    workspace: WorkspaceObservation,
    settings: dict[str, Any],
) -> dict[str, Any] | None:
    if workspace.homography is None:
        return None
    robot_polygon = _workspace_robot_polygon(settings)
    if robot_polygon is None:
        return None
    min_x = float(np.min(robot_polygon[:, 0]))
    max_x = float(np.max(robot_polygon[:, 0]))
    min_y = float(np.min(robot_polygon[:, 1]))
    max_y = float(np.max(robot_polygon[:, 1]))
    width_mm = max_x - min_x
    height_mm = max_y - min_y
    if width_mm <= 1e-6 or height_mm <= 1e-6:
        return None

    pixels_per_mm = max(0.5, min(4.0, float(settings.get("projection_pixels_per_mm", 1.0))))
    width_px = max(32, min(1600, int(round(width_mm * pixels_per_mm)) + 1))
    height_px = max(32, min(1600, int(round(height_mm * pixels_per_mm)) + 1))
    scale_x = (width_px - 1) / width_mm
    scale_y = (height_px - 1) / height_mm
    robot_to_texture = np.array(
        [
            [scale_x, 0.0, -min_x * scale_x],
            [0.0, -scale_y, max_y * scale_y],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    image_to_texture = robot_to_texture @ workspace.homography
    warped = cv2.warpPerspective(
        image_bgr,
        image_to_texture,
        (width_px, height_px),
        flags=cv2.INTER_LINEAR,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(0, 0, 0),
    )

    texture_polygon_float = np.column_stack(
        (
            (robot_polygon[:, 0] - min_x) * scale_x,
            (max_y - robot_polygon[:, 1]) * scale_y,
        )
    )
    jpeg_quality = max(
        40,
        min(95, int(settings.get("projection_jpeg_quality", 82))),
    )

    return {
        "ok": True,
        "image_b64": encode_image_b64(
            warped,
            ".jpg",
            [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality],
        ),
        "homography_source": workspace.homography_source,
        "workspace_polygon_source": workspace.polygon_source,
        "robot_bounds_mm": {
            "min_x": min_x,
            "max_x": max_x,
            "min_y": min_y,
            "max_y": max_y,
            "z": 1.2,
        },
        "robot_polygon_mm": robot_polygon.tolist(),
        "texture_polygon_px": texture_polygon_float.tolist(),
        "projection_mode": "workspace_polygon",
        "texture_size_px": {"width": width_px, "height": height_px},
    }


def _workspace_color_candidates(
    image_bgr: np.ndarray,
    workspace_polygon_px: np.ndarray | None,
    settings: dict[str, Any],
) -> tuple[list[dict[str, Any]], np.ndarray]:
    blurred = cv2.GaussianBlur(image_bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    lower = np.array(
        [0, int(settings.get("min_saturation", 60)), int(settings.get("min_value", 50))],
        dtype=np.uint8,
    )
    upper = np.array([179, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)
    workspace_only = bool(settings.get("workspace_only", True))
    polygon_mask = _workspace_mask(image_bgr.shape[:2], workspace_polygon_px)
    if polygon_mask is not None:
        mask = cv2.bitwise_and(mask, polygon_mask)
    elif workspace_only:
        return [], np.zeros(image_bgr.shape[:2], dtype=np.uint8)

    kernel_size = max(1, int(settings.get("morph_kernel_px", 5)))
    if kernel_size % 2 == 0:
        kernel_size += 1
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    if polygon_mask is not None:
        mask = cv2.bitwise_and(mask, polygon_mask)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    minimum_area = max(1.0, float(settings.get("min_object_area_px", 400.0)))
    candidates: list[dict[str, Any]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        if area < minimum_area:
            continue
        moments = cv2.moments(contour)
        if abs(moments["m00"]) < 1e-9:
            continue
        center_x = float(moments["m10"] / moments["m00"])
        center_y = float(moments["m01"] / moments["m00"])
        x, y, width, height = cv2.boundingRect(contour)
        object_mask = np.zeros(mask.shape, dtype=np.uint8)
        cv2.drawContours(object_mask, [contour], -1, 255, -1)
        candidates.append(
            {
                "label": _classify_color(hsv[object_mask == 255]),
                "center_px": (center_x, center_y),
                "bbox_px": (int(x), int(y), int(width), int(height)),
                "area_px": area,
                "confidence": min(1.0, area / (minimum_area * 4.0)),
            }
        )
    candidates.sort(key=lambda item: (item["center_px"][0], item["center_px"][1]))
    return candidates, mask


@dataclass
class WorkspaceObservation:
    enabled: bool
    dictionary: str
    detection_mode: str
    visible_ids: list[int]
    required_ids: list[int]
    polygon_px: np.ndarray | None
    homography: np.ndarray | None
    homography_source: str
    error: str | None
    marker_detection: FiducialDetection
    configured_dictionary: str | None = None
    candidate_dictionaries: list[str] | None = None
    polygon_source: str = "none"
    warning: str | None = None
    layout_mismatches: list[str] | None = None
    layout_errors_px: dict[int, float] | None = None
    homography_metrics: dict[str, Any] | None = None

    @property
    def corners(self) -> list[np.ndarray]:
        return self.marker_detection.corners

    @property
    def ids(self) -> np.ndarray | None:
        return self.marker_detection.ids

    def to_dict(self) -> dict[str, Any]:
        visible = set(self.visible_ids)
        required = set(self.required_ids)
        tags_observed = self.detection_mode not in {"not_checked", "disabled"}
        missing_ids = sorted(required - visible) if tags_observed else []
        all_required_visible = required.issubset(visible)
        if not self.enabled:
            status = "disabled"
            message = "Workspace calibration is disabled."
        elif self.homography_source == "workspace_aruco_saved":
            status = "saved_calibration"
            message = "Using the saved planar workspace calibration."
            if self.warning:
                message = f"{message} {self.warning}"
        elif self.homography is not None:
            status = "calibrated"
            message = "Workspace planar calibration is active."
        else:
            status = "uncalibrated"
            message = self.error or "No usable workspace calibration is available."
        return {
            "enabled": self.enabled,
            "dictionary": self.dictionary,
            "configured_dictionary": self.configured_dictionary or self.dictionary,
            "candidate_dictionaries": self.candidate_dictionaries or [self.dictionary],
            "detection_mode": self.detection_mode,
            "visible_ids": self.visible_ids,
            "required_ids": self.required_ids,
            "missing_ids": missing_ids,
            "all_required_visible": all_required_visible,
            "workspace_polygon_px": (
                self.polygon_px.astype(float).tolist() if self.polygon_px is not None else None
            ),
            "workspace_polygon_source": self.polygon_source,
            "detection_source": "full_frame" if tags_observed else "not_checked",
            "homography_source": self.homography_source,
            "homography_metrics": self.homography_metrics or {},
            "calibrated": self.homography is not None,
            "status": status,
            "message": message,
            "warning": self.warning,
            "layout_mismatches": self.layout_mismatches or [],
            "layout_errors_px": (
                {str(key): float(value) for key, value in self.layout_errors_px.items()}
                if self.layout_errors_px
                else {}
            ),
            "live_tags_required": False,
            "error": self.error,
        }


class VisionPipeline:
    """Planar workspace/color pipeline with a detector-neutral output contract."""

    def __init__(self) -> None:
        self._last_workspace_polygon: np.ndarray | None = None
        self._last_image_shape: tuple[int, int] | None = None
        self._signature: tuple[Any, ...] | None = None

    @staticmethod
    def _workspace_signature(camera: dict[str, Any]) -> tuple[Any, ...]:
        settings = workspace_aruco_settings(camera)
        resolution = camera.get("resolution") if isinstance(camera.get("resolution"), dict) else {}
        return (
            int(camera.get("source_index", 0)),
            int(resolution.get("width", 0) or 0),
            int(resolution.get("height", 0) or 0),
            str(settings.get("dictionary", "")),
            repr(settings.get("dictionary_candidates", [])),
            repr(settings.get("tag_box_corner_index", {})),
            repr(settings.get("tag_centers_robot_mm", {})),
            repr(settings.get("reference_points_px", {})),
            repr(settings.get("reference_workspace_corners_px", {})),
            repr(settings.get("workspace_polygon_robot_mm", [])),
        )

    def configure(self, camera: dict[str, Any]) -> None:
        signature = self._workspace_signature(camera)
        if signature != self._signature:
            self._last_workspace_polygon = None
            self._last_image_shape = None
            self._signature = signature

    def _observe_saved_workspace(
        self,
        image_bgr: np.ndarray,
        camera: dict[str, Any],
    ) -> WorkspaceObservation:
        self.configure(camera)
        settings = workspace_aruco_settings(camera)
        required = [int(value) for value in settings.get("required_ids", [])]
        configured_dictionary = str(settings.get("dictionary", "DICT_4X4_50")).upper()
        candidates = dictionary_candidates(settings)
        empty_detection = FiducialDetection(
            [],
            None,
            configured_dictionary,
            "not_checked",
            [],
            0,
        )
        if not bool(settings.get("enabled", False)):
            return WorkspaceObservation(
                enabled=False,
                dictionary=configured_dictionary,
                detection_mode="disabled",
                visible_ids=[],
                required_ids=required,
                polygon_px=None,
                homography=None,
                homography_source="none",
                error="workspace ArUco calibration is disabled",
                marker_detection=FiducialDetection(
                    [],
                    None,
                    configured_dictionary,
                    "disabled",
                    [],
                    0,
                ),
                configured_dictionary=configured_dictionary,
                candidate_dictionaries=candidates,
            )

        homography, metrics = saved_homography(settings, image_bgr.shape)
        source = "workspace_aruco_saved" if homography is not None else "none"
        error = None if homography is not None else str(
            metrics.get("error") or "no saved workspace calibration is available"
        )
        polygon = (
            polygon_from_robot(homography, settings)
            if homography is not None
            else None
        )
        polygon_source = "saved_workspace_config" if polygon is not None else "none"

        return WorkspaceObservation(
            enabled=True,
            dictionary=configured_dictionary,
            detection_mode="not_checked",
            visible_ids=[],
            required_ids=required,
            polygon_px=polygon,
            homography=homography,
            homography_source=source,
            error=error,
            marker_detection=empty_detection,
            configured_dictionary=configured_dictionary,
            candidate_dictionaries=candidates,
            polygon_source=polygon_source,
            homography_metrics=metrics,
        )

    def process(
        self,
        image_bgr: np.ndarray,
        camera: dict[str, Any],
        profiles: dict[str, dict[str, Any]],
        profile_names: list[str] | None = None,
        include_workspace_projection: bool = False,
    ) -> dict[str, Any]:
        workspace = self._observe_saved_workspace(image_bgr, camera)
        calibration = camera.get("calibration") if isinstance(camera.get("calibration"), dict) else {}
        if workspace.homography is not None:
            projection = {
                "source": workspace.homography_source,
                "homography": workspace.homography,
            }
        else:
            projection = _projection_from_calibration(calibration)
        detection_settings = (
            camera.get("detection") if isinstance(camera.get("detection"), dict) else {}
        )
        provider = str(detection_settings.get("provider", "workspace_color")).lower()
        selected_profiles = profiles
        if profile_names:
            selected = set(profile_names)
            selected_profiles = {name: profile for name, profile in profiles.items() if name in selected}

        if provider == "configured_hsv":
            detections = detect_configured_colors(image_bgr, selected_profiles, calibration, projection)
            mask = np.zeros(image_bgr.shape[:2], dtype=np.uint8)
        else:
            candidates, mask = _workspace_color_candidates(
                image_bgr,
                workspace.polygon_px,
                detection_settings,
            )
            detections = []
            show_unconfigured = bool(detection_settings.get("show_unconfigured_colors", True))
            for index, candidate in enumerate(candidates, start=1):
                label = str(candidate["label"])
                profile = selected_profiles.get(label)
                if profile is None and not show_unconfigured:
                    continue
                if profile is not None and profile.get("enabled", True) is False:
                    continue
                minimum = max(
                    1.0,
                    float(
                        (profile or {}).get(
                            "min_area_px",
                            detection_settings.get("min_object_area_px", 400.0),
                        )
                    ),
                )
                if candidate["area_px"] < minimum:
                    continue
                detections.append(
                    _detection_contract(
                        label=label,
                        center_px=candidate["center_px"],
                        bbox_px=candidate["bbox_px"],
                        area_px=candidate["area_px"],
                        projection=projection,
                        confidence=max(
                            candidate["confidence"],
                            min(1.0, candidate["area_px"] / (minimum * 4.0)),
                        ),
                        detector="workspace_color",
                        drop_zone=(profile or {}).get("drop_zone"),
                        task_eligible=bool(profile and profile.get("enabled", True)),
                        detection_id=f"{label}-{index}",
                    )
                )
            detections.sort(
                key=lambda item: (
                    float((item.get("robot") or {}).get("x_mm", item.get("center_px", {}).get("x", 0.0))),
                    float((item.get("robot") or {}).get("y_mm", item.get("center_px", {}).get("y", 0.0))),
                )
            )
            for object_index, detection in enumerate(detections, start=1):
                detection["object_id"] = object_index

        annotated = annotated_detection_frame(image_bgr, detections, workspace=workspace)
        workspace_payload = workspace.to_dict()
        workspace_payload["image_size_px"] = {
            "width": int(image_bgr.shape[1]),
            "height": int(image_bgr.shape[0]),
        }
        response = {
            "ok": True,
            "detections": detections,
            "annotated": annotated,
            "mask": mask,
            "workspace": workspace_payload,
            "provider": provider,
            "calibration_source": projection.get("source", "unavailable"),
        }
        if include_workspace_projection:
            response["workspace_projection"] = _workspace_projection_texture(
                image_bgr,
                workspace,
                workspace_aruco_settings(camera),
            )
        return response

    def project_workspace_frame(
        self,
        image_bgr: np.ndarray,
        camera: dict[str, Any],
    ) -> dict[str, Any]:
        workspace = self._observe_saved_workspace(image_bgr, camera)
        projection = _workspace_projection_texture(
            image_bgr,
            workspace,
            workspace_aruco_settings(camera),
        )
        return {
            "ok": projection is not None,
            "workspace": {
                **workspace.to_dict(),
                "image_size_px": {
                    "width": int(image_bgr.shape[1]),
                    "height": int(image_bgr.shape[0]),
                },
            },
            "workspace_projection": projection,
            "error": workspace.error if projection is None else None,
        }

    def project_external_detections(
        self,
        detections: list[dict[str, Any]],
        camera: dict[str, Any],
    ) -> list[dict[str, Any]]:
        calibration = camera.get("calibration") if isinstance(camera.get("calibration"), dict) else {}
        settings = workspace_aruco_settings(camera)
        resolution = camera.get("resolution") if isinstance(camera.get("resolution"), dict) else {}
        shape = (
            int(resolution.get("height", 0) or 0),
            int(resolution.get("width", 0) or 0),
            3,
        )
        homography = None
        if shape[0] > 0 and shape[1] > 0:
            homography, _metrics = saved_homography(settings, shape)
        projection = (
            {"source": "workspace_aruco_saved", "homography": homography}
            if homography is not None
            else _projection_from_calibration(calibration)
        )

        normalized: list[dict[str, Any]] = []
        for index, raw in enumerate(detections, start=1):
            center = raw.get("center_px") if isinstance(raw.get("center_px"), dict) else {}
            bbox = raw.get("bbox_px") if isinstance(raw.get("bbox_px"), dict) else {}
            cx = float(raw.get("cx", center.get("x", 0.0)))
            cy = float(raw.get("cy", center.get("y", 0.0)))
            width = int(raw.get("width", bbox.get("w", 0)))
            height = int(raw.get("height", bbox.get("h", 0)))
            x = int(raw.get("x", bbox.get("x", round(cx - width / 2))))
            y = int(raw.get("y", bbox.get("y", round(cy - height / 2))))
            normalized.append(
                _detection_contract(
                    label=str(raw.get("label", raw.get("class_name", "object"))),
                    center_px=(cx, cy),
                    bbox_px=(x, y, width, height),
                    area_px=float(raw.get("area_px", max(width * height, 0))),
                    projection=projection,
                    confidence=float(raw.get("confidence", 0.0)),
                    detector=str(raw.get("detector", "external_ai")),
                    drop_zone=raw.get("drop_zone"),
                    task_eligible=bool(raw.get("task_eligible", True)),
                    detection_id=str(raw.get("id", f"external-{index}")),
                )
            )
        return normalized


def annotated_detection_frame(
    image_bgr: np.ndarray,
    detections: list[dict[str, Any]],
    workspace: WorkspaceObservation | None = None,
) -> np.ndarray:
    annotated = image_bgr.copy()
    palette = {
        "red": (30, 30, 230),
        "orange": (0, 135, 255),
        "yellow": (0, 220, 230),
        "green": (70, 200, 70),
        "cyan": (220, 220, 40),
        "blue": (230, 90, 30),
        "purple": (210, 80, 180),
        "pink": (180, 80, 240),
    }
    if workspace is not None:
        if workspace.ids is not None and workspace.corners:
            cv2.aruco.drawDetectedMarkers(
                annotated,
                [corner.astype(np.float32).reshape(1, 4, 2) for corner in workspace.corners],
                workspace.ids,
            )
        if workspace.polygon_px is not None:
            cv2.polylines(
                annotated,
                [np.round(workspace.polygon_px).astype(np.int32)],
                True,
                (0, 220, 255),
                2,
            )
    for detection in detections:
        if not detection.get("ok"):
            continue
        label_name = str(detection.get("label", detection.get("color", "object")))
        color = palette.get(label_name, (230, 230, 230))
        bbox = detection.get("bbox_px") or {}
        center = detection.get("center_px") or {}
        x = int(bbox.get("x", center.get("x", 0)))
        y = int(bbox.get("y", center.get("y", 0)))
        width = int(bbox.get("w", 0))
        height = int(bbox.get("h", 0))
        cx = int(center.get("x", x + width / 2))
        cy = int(center.get("y", y + height / 2))
        if width > 0 and height > 0:
            cv2.rectangle(annotated, (x, y), (x + width, y + height), color, 2)
        cv2.circle(annotated, (cx, cy), 4, color, -1)
        robot = detection.get("robot") or {}
        label = f"{detection.get('id', '')} {label_name} px({cx},{cy})".strip()
        if robot:
            label += f" robot({robot.get('x_mm', 0):.1f},{robot.get('y_mm', 0):.1f})"
        cv2.putText(
            annotated,
            label,
            (max(0, x), max(16, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            1,
            cv2.LINE_AA,
        )
    status = "Vision ready"
    if workspace is not None:
        if workspace.warning:
            status = f"tag layout warning | visible {workspace.visible_ids or '-'} | using saved calibration"
        elif workspace.homography_source == "workspace_aruco_saved":
            status = (
                f"saved planar calibration | workspace {workspace.polygon_source}"
            )
        else:
            status = (
                f"{workspace.dictionary} {workspace.detection_mode} | "
                f"tags {workspace.visible_ids or '-'} | {workspace.homography_source}"
            )
    cv2.rectangle(
        annotated,
        (8, 8),
        (min(annotated.shape[1] - 8, 760), 40),
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
