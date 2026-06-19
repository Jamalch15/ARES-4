# Vision Integration

## Status

This documents the current working integration of `vision_robot_project.zip`.
The camera geometry and thresholds remain setup-specific assumptions, not final
robot calibration.

## Current Pipeline

```text
USB camera
-> saved workspace polygon and planar pixel-to-robot homography
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

## Workspace Calibration

`camera.calibration.workspace_aruco` is the authoritative operator calibration.
It uses the four physical tag corners facing away from the workspace center,
independent of each printed marker's rotation. Repeated observations are
median-filtered before the four outer corners define the image-to-robot
homography. Tag centers remain layout and ID diagnostics, not fit points.
Tag detection and homography solving run only when the operator presses
`Calibrate workspace` or `Verify calibration`. Normal vision and viewport
streaming use the saved homography without live recalibration.

The robot frame is:

- `X`: sideways across the work plate
- `Y`: forward from the robot base
- `Z`: upward, with the workspace at `Z=0`

The saved map drives color-object coordinates, external AI detection
coordinates, the workspace mask, and the projected camera texture. Camera
intrinsics are not part of this path. The older `apriltag` 6-DoF implementation
is retained only as an optional developer feature and is hidden from normal
settings.

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
- Re-run workspace calibration whenever the physical
  camera, resolution, zoom, or tags move.
- Add editable color profiles and physical workspace-bound validation in the
  Tasks workflow. Settings should remain focused on camera and calibration.
