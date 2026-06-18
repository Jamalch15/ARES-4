# Vision Integration

## Status

This documents the current working integration of `vision_robot_project.zip`.
The camera geometry and thresholds remain setup-specific assumptions, not final
robot calibration.

## Current Pipeline

```text
USB camera
-> normal/inverted square-tag detection
-> workspace polygon and planar pixel-to-robot homography
-> workspace-masked multi-object color detection
-> normalized detection objects
-> task preview
-> IK/path validation
-> operator-confirmed execution
```

The ZIP assumes camera index `1` at `640 x 480`. Hardware enumeration on
2026-06-18 found the current table camera at index `2`; index `1` is EOS
Webcam Utility at `1280 x 720`, and index `0` is a disabled/blocked camera
placeholder. The ignored local configuration therefore uses index `2`, while
the example keeps the imported ZIP assumption visible. Saved ArUco reference
points are valid only while the camera, resolution, zoom, and work plate remain
fixed.

## Calibration Paths

The two calibration paths are intentionally separate:

- `workspace_aruco`: the imported, known-working 2D homography using inverted
  `DICT_4X4_50` IDs 0-3. The marker backend can also try
  `DICT_APRILTAG_36H11` because the physical table tags may use either family.
  This is the current default for table objects and does not require camera
  intrinsics.
- `apriltag`: a tag-based calibration workflow that can save either a planar
  image-to-robot homography or, when real camera intrinsics are configured, a
  full 6-DoF camera pose. Intrinsics are only required for the 3D pose path.

An accepted AprilTag 3D pose has projection priority. A saved AprilTag planar
homography, the working ArUco homography, and the older manually entered
four-point homography are the planar fallback paths.
For the current temporary physical setup, the reference tags may be absent. In
that case the app treats the saved ZIP homography and saved workspace polygon as
the normal calibrated task mode; live tags become optional diagnostics until the
tags are glued on.

## Detection Contract

Vision providers return a common object shape containing:

- `id`, `label`, `confidence`, `timestamp`
- `center_px`, `bbox_px`, and `area_px`
- optional calibrated `robot` coordinates
- `coordinate_source` and projection diagnostics
- optional `drop_zone` and `task_eligible`

Task code consumes this contract and does not depend on OpenCV contour details.

## AI Integration Boundary

No trained AI model is included yet. Future YOLO or other inference code should
send image-space detections to `POST /api/vision/project`. The backend projects
and normalizes them into the same contract used by the color detector, so task,
IK, motion, and frontend code do not need model-specific branches.

## Open Questions

- Confirm camera enumeration on the demonstration PC; indices can change when
  virtual cameras or USB devices are added.
- Re-record ArUco reference points if the physical setup has moved.
- Confirm the imported robot XY tag-center coordinates against the robot base
  frame used by motion planning.
- Add a guided reference-point recapture workflow instead of relying only on
  the imported saved points.
- Add editable color profiles and physical workspace-bound validation in the
  Tasks workflow. Settings should remain focused on camera and calibration.
