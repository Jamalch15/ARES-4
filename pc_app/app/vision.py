from __future__ import annotations

import base64
from typing import Any

import cv2
import numpy as np


def planar_transform_from_points(
    image_points: list[list[float]],
    robot_points: list[list[float]],
) -> np.ndarray:
    if len(image_points) != 4 or len(robot_points) != 4:
        raise ValueError("planar calibration requires exactly four image and robot points")
    src = np.array(image_points, dtype=np.float32)
    dst = np.array(robot_points, dtype=np.float32)
    return cv2.getPerspectiveTransform(src, dst)


def apply_planar_transform(transform: np.ndarray, image_point: list[float]) -> dict[str, float]:
    point = np.array([[[float(image_point[0]), float(image_point[1])]]], dtype=np.float32)
    mapped = cv2.perspectiveTransform(point, transform)[0][0]
    return {"x_mm": float(mapped[0]), "y_mm": float(mapped[1])}


def decode_image_b64(image_b64: str) -> np.ndarray:
    payload = image_b64.split(",", 1)[-1]
    raw = base64.b64decode(payload)
    image = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("could not decode image")
    return image


def detect_color_blob(image_bgr: np.ndarray, profile: dict[str, Any]) -> dict[str, Any]:
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
    x, y, w, h = cv2.boundingRect(contour)
    return {
        "ok": True,
        "center_px": {"x": cx, "y": cy},
        "area_px": area,
        "bbox_px": {"x": int(x), "y": int(y), "w": int(w), "h": int(h)},
    }


def detect_configured_colors(
    image_bgr: np.ndarray,
    profiles: dict[str, dict[str, Any]],
    calibration: dict[str, Any] | None = None,
) -> list[dict[str, Any]]:
    transform = None
    if calibration:
        image_points = calibration.get("image_points") or []
        robot_points = calibration.get("robot_points") or []
        if len(image_points) == 4 and len(robot_points) == 4:
            transform = planar_transform_from_points(image_points, robot_points)

    detections: list[dict[str, Any]] = []
    for name, profile in profiles.items():
        if not bool(profile.get("enabled", True)):
            continue
        result = detect_color_blob(image_bgr, profile)
        result["color"] = name
        result["drop_zone"] = profile.get("drop_zone")
        if result.get("ok") and transform is not None:
            center = result["center_px"]
            robot_xy = apply_planar_transform(transform, [center["x"], center["y"]])
            result["robot"] = robot_xy
        detections.append(result)
    return detections

