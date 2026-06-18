from __future__ import annotations

import base64

import cv2
import numpy as np
from fastapi.testclient import TestClient
from pytest import approx

from app import main
from app.config import load_config
from app.demo_settings import camera_settings, color_profiles
from app.vision import VisionPipeline


def working_camera() -> dict:
    camera = camera_settings(load_config())
    camera["enabled"] = True
    camera["source_index"] = 1
    camera["resolution"] = {"width": 640, "height": 480}
    camera["detection"] = {**camera.get("detection", {}), "provider": "workspace_color"}
    return camera


def paste_aruco_marker(
    image: np.ndarray,
    dictionary_name: str,
    marker_id: int,
    center_px: tuple[float, float],
    size_px: int = 58,
) -> None:
    dictionary_id = getattr(cv2.aruco, dictionary_name)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    marker = cv2.aruco.generateImageMarker(dictionary, marker_id, size_px)
    marker_bgr = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)
    x0 = int(round(center_px[0] - size_px / 2))
    y0 = int(round(center_px[1] - size_px / 2))
    image[y0 : y0 + size_px, x0 : x0 + size_px] = marker_bgr


def test_workspace_pipeline_detects_multiple_objects_and_masks_outside_workspace():
    camera = working_camera()
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.circle(image, (260, 220), 18, (0, 0, 255), -1)
    cv2.circle(image, (380, 250), 20, (0, 0, 255), -1)
    cv2.circle(image, (20, 20), 18, (0, 0, 255), -1)

    result = VisionPipeline().process(image, camera, color_profiles(load_config()))

    assert result["ok"]
    assert result["workspace"]["homography_source"] == "workspace_aruco_saved"
    assert result["workspace"]["status"] == "saved_calibration"
    assert result["workspace"]["live_tags_required"] is False
    assert result["workspace"]["workspace_polygon_source"] == "saved_workspace_config"
    assert len(result["detections"]) == 2
    assert all(detection["label"] == "red" for detection in result["detections"])
    assert all(detection["task_eligible"] for detection in result["detections"])
    assert all(detection["coordinate_source"] == "workspace_aruco_saved" for detection in result["detections"])
    assert all(detection["robot"]["z_mm"] == 0.0 for detection in result["detections"])
    assert [detection["object_id"] for detection in result["detections"]] == [1, 2]
    assert all({"id", "confidence", "center_px", "bbox_px", "timestamp"} <= detection.keys() for detection in result["detections"])


def test_workspace_tag_detection_falls_back_to_apriltag_dictionary():
    camera = working_camera()
    settings = camera["calibration"]["workspace_aruco"]
    settings["dictionary"] = "DICT_4X4_50"
    settings["dictionary_candidates"] = ["DICT_4X4_50", "DICT_APRILTAG_36H11"]
    image = np.full((480, 640, 3), 220, dtype=np.uint8)

    for marker_id, center in settings["reference_points_px"].items():
        paste_aruco_marker(
            image,
            "DICT_APRILTAG_36H11",
            int(marker_id),
            (float(center[0]), float(center[1])),
        )

    result = VisionPipeline().process(image, camera, color_profiles(load_config()))
    workspace = result["workspace"]

    assert result["ok"]
    assert workspace["dictionary"] == "DICT_APRILTAG_36H11"
    assert workspace["configured_dictionary"] == "DICT_4X4_50"
    assert workspace["detection_mode"] == "normal"
    assert workspace["visible_ids"] == [0, 1, 2, 3]
    assert workspace["homography_source"] == "workspace_aruco_live"
    assert workspace["status"] == "live_tags"
    assert workspace["workspace_polygon_source"] == "live_tags"
    assert workspace["calibrated"] is True


def test_live_workspace_tags_with_wrong_layout_fall_back_to_saved_calibration():
    camera = working_camera()
    settings = camera["calibration"]["workspace_aruco"]
    image = np.full((480, 640, 3), 220, dtype=np.uint8)
    swapped_layout = {
        3: settings["reference_points_px"]["0"],
        1: settings["reference_points_px"]["1"],
        2: settings["reference_points_px"]["2"],
        0: settings["reference_points_px"]["3"],
    }

    for marker_id, center in swapped_layout.items():
        paste_aruco_marker(
            image,
            "DICT_4X4_50",
            marker_id,
            (float(center[0]), float(center[1])),
        )

    result = VisionPipeline().process(image, camera, color_profiles(load_config()))
    workspace = result["workspace"]

    assert workspace["visible_ids"] == [0, 1, 2, 3]
    assert workspace["homography_source"] == "workspace_aruco_saved"
    assert workspace["status"] == "saved_calibration"
    assert workspace["warning"]
    assert workspace["layout_mismatches"]


def test_configured_hsv_provider_uses_workspace_projection_when_available():
    camera = working_camera()
    camera["detection"]["provider"] = "configured_hsv"
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.circle(image, (260, 220), 18, (0, 0, 255), -1)

    result = VisionPipeline().process(image, camera, color_profiles(load_config()))
    red = next(detection for detection in result["detections"] if detection["label"] == "red")

    assert red["ok"]
    assert red["coordinate_source"] == "workspace_aruco_saved"
    assert red["robot"] is not None
    assert red["robot"]["z_mm"] == 0.0


def test_workspace_projection_texture_uses_saved_homography_bounds():
    camera = working_camera()
    image = np.full((480, 640, 3), 80, dtype=np.uint8)
    cv2.circle(image, (260, 220), 18, (0, 0, 255), -1)

    result = VisionPipeline().process(
        image,
        camera,
        color_profiles(load_config()),
        include_workspace_projection=True,
    )
    projection = result["workspace_projection"]

    assert projection["ok"]
    assert projection["homography_source"] == "workspace_aruco_saved"
    assert projection["workspace_polygon_source"] == "saved_workspace_config"
    assert projection["image_b64"].startswith("data:image/png;base64,")
    assert projection["texture_size_px"] == {"width": 464, "height": 794}
    assert projection["robot_bounds_mm"] == {
        "min_x": approx(131.5),
        "max_x": approx(363.5),
        "min_y": approx(-198.5),
        "max_y": approx(198.5),
        "z": approx(1.2),
    }

    raw = base64.b64decode(projection["image_b64"].split(",", 1)[1])
    texture = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    assert texture.shape == (794, 464, 4)
    assert int(texture[:, :, 3].max()) == 220


def test_external_ai_detections_use_the_same_projection_contract():
    pipeline = VisionPipeline()

    detections = pipeline.project_external_detections(
        [
            {
                "id": "yolo-7",
                "label": "red",
                "confidence": 0.91,
                "cx": 260.0,
                "cy": 220.0,
                "width": 36,
                "height": 40,
            }
        ],
        working_camera(),
    )

    assert len(detections) == 1
    detection = detections[0]
    assert detection["id"] == "yolo-7"
    assert detection["detector"] == "external_ai"
    assert detection["confidence"] == approx(0.91)
    assert detection["coordinate_source"] == "workspace_aruco_saved"
    assert detection["robot"]["x_mm"] == approx(237.57, abs=0.1)
    assert detection["robot"]["y_mm"] == approx(-58.75, abs=0.1)


def test_external_ai_projection_api_exposes_normalized_detections():
    client = TestClient(main.app)

    response = client.post(
        "/api/vision/project",
        json={
            "detections": [
                {
                    "label": "blue",
                    "confidence": 0.8,
                    "center_px": {"x": 380, "y": 250},
                    "bbox_px": {"x": 360, "y": 230, "w": 40, "h": 40},
                }
            ]
        },
    )

    payload = response.json()
    assert payload["ok"]
    assert payload["provider"] == "external_ai"
    assert payload["detections"][0]["label"] == "blue"
    assert payload["detections"][0]["robot"] is not None
