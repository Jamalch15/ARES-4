from __future__ import annotations

import numpy as np
from pytest import approx

from app.config import load_config
from app.demo_settings import camera_settings
from app.vision import workspace_aruco_settings
from app.workspace_calibration import (
    FiducialDetection,
    WorkspaceCalibrationSession,
    marker_box_corners,
    saved_homography,
    workspace_mapping_errors,
)


def robot_to_image(point: list[float] | np.ndarray) -> np.ndarray:
    return np.asarray([float(point[0]) + 320.0, 450.0 - float(point[1])])


def exact_detection(settings: dict) -> FiducialDetection:
    ids = [int(value) for value in settings["required_ids"]]
    robot_centers = settings["tag_centers_robot_mm"]
    robot_polygon = np.asarray(settings["workspace_polygon_robot_mm"], dtype=np.float64)
    corners = []
    for marker_id in ids:
        center_robot = np.asarray(robot_centers[str(marker_id)], dtype=np.float64)
        center_image = robot_to_image(center_robot)
        workspace_corner_robot = robot_polygon[
            int(np.argmin(np.linalg.norm(robot_polygon - center_robot, axis=1)))
        ]
        selected_vector = robot_to_image(workspace_corner_robot) - center_image
        perpendicular = np.asarray([-selected_vector[1], selected_vector[0]])
        vectors = [
            selected_vector,
            perpendicular,
            -selected_vector,
            -perpendicular,
        ]
        selected_index = int(settings["tag_box_corner_index"][str(marker_id)])
        ordered = [None, None, None, None]
        for offset, vector in enumerate(vectors):
            ordered[(selected_index + offset) % 4] = center_image + vector
        corners.append(np.asarray(ordered, dtype=np.float32))
    return FiducialDetection(
        corners=corners,
        ids=np.asarray(ids, dtype=np.int32).reshape(-1, 1),
        dictionary="DICT_4X4_50",
        mode="normal",
        visible_ids=ids,
    )


def test_workspace_session_solves_outer_corners_without_intrinsics():
    settings = workspace_aruco_settings(camera_settings(load_config()))
    session = WorkspaceCalibrationSession(settings)
    detection = exact_detection(settings)

    for _ in range(settings["minimum_samples"]):
        session.add(detection, (480, 640, 3))

    result = session.solve(require_minimum_samples=True)

    assert result["ok"]
    assert result["ready"]
    assert result["quality_ok"]
    assert result["metrics"]["point_count"] == 4
    assert result["metrics"]["fit_source"] == "workspace_outer_corners"
    assert result["metrics"]["tag_center_point_count"] == 0
    assert result["metrics"]["workspace_corner_point_count"] == 4
    assert result["metrics"]["rmse_mm"] == approx(0.0, abs=1e-6)
    assert result["metrics"]["tag_center_max_error_mm"] == approx(0.0, abs=1e-6)
    assert result["reference_resolution"] == {"width": 640, "height": 480}


def test_saved_workspace_mapping_verifies_robot_xy_coordinates():
    settings = workspace_aruco_settings(camera_settings(load_config()))
    session = WorkspaceCalibrationSession(settings)
    detection = exact_detection(settings)
    for _ in range(settings["minimum_samples"]):
        session.add(detection, (480, 640, 3))
    result = session.solve(require_minimum_samples=True)
    settings.update(
        {
            "reference_points_px": result["reference_points_px"],
            "reference_workspace_corners_px": result[
                "reference_workspace_corners_px"
            ],
            "reference_resolution": result["reference_resolution"],
        }
    )

    homography, metrics = saved_homography(settings, (480, 640, 3))
    verification = workspace_mapping_errors(homography, detection, settings)

    assert metrics["ok"]
    assert verification["ok"]
    assert verification["point_count"] == 4
    assert verification["corner_point_count"] == 4
    assert verification["rmse_mm"] == approx(0.0, abs=1e-6)


def test_workspace_outer_corners_ignore_each_markers_printed_rotation():
    geometric_corners = [
        np.asarray([[70, 60], [120, 60], [120, 110], [70, 110]], dtype=np.float32),
        np.asarray([[520, 65], [570, 65], [570, 115], [520, 115]], dtype=np.float32),
        np.asarray([[72, 345], [122, 345], [122, 395], [72, 395]], dtype=np.float32),
        np.asarray([[518, 350], [568, 350], [568, 400], [518, 400]], dtype=np.float32),
    ]
    rotations = [0, 1, 2, 3]
    rotated = [
        np.roll(marker_corners, shift, axis=0)
        for marker_corners, shift in zip(geometric_corners, rotations, strict=True)
    ]
    ids = np.asarray([[0], [1], [2], [3]], dtype=np.int32)

    selected = marker_box_corners(rotated, ids, {})

    assert selected[0] == approx([70, 60])
    assert selected[1] == approx([570, 65])
    assert selected[2] == approx([72, 395])
    assert selected[3] == approx([568, 400])


def test_workspace_session_rejects_tag_centers_in_the_wrong_layout():
    settings = workspace_aruco_settings(camera_settings(load_config()))
    detection = exact_detection(settings)
    detection.corners[0] = detection.corners[0] + np.asarray(
        [120.0, 90.0],
        dtype=np.float32,
    )
    session = WorkspaceCalibrationSession(settings)
    for _ in range(settings["minimum_samples"]):
        session.add(detection, (480, 640, 3))

    result = session.solve(require_minimum_samples=True)

    assert not result["ok"]
    assert not result["quality_ok"]
    assert result["metrics"]["tag_center_max_error_mm"] > 12.0
