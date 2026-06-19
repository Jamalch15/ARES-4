# Camera Settings And Workspace Calibration

## Current Design

The operator workflow is planar and does not require camera intrinsics.

1. Camera source settings select the enabled camera, index, and resolution.
2. Pressing `Calibrate workspace` collects stable observations of all four
   tags, solves the outer-corner homography, validates tag-center layout, and
   saves the result as one operation.
3. Normal operation never re-solves from live tags. It uses only the saved map
   for robot coordinates, workspace masking, and camera projection.
4. `Verify calibration` compares a fresh tag frame against the saved map.
5. The 3D viewport continuously warps raw camera frames through the saved map.

Robot coordinates use `X` sideways and `Y` forward. The configured outer
workspace polygon is:

```text
(-239.0, 86.5) -> (239.0, 86.5)
       |                    |
(-239.0, 401.5) <- (239.0, 401.5)
```

## Settings UI

The Camera settings section now contains:

- camera enable, index, width, and height
- planar workspace mapping enable
- projected workspace camera enable
- normal/inverted tag detection options
- one calibrate action and one verification action
- an annotated frame showing the exact outer workspace outline
- per-tag pixel and robot-coordinate diagnostics
- planar fit and fresh verification error in millimeters

The Settings preview updates only after calibration or verification. Color
detection and its live camera popup remain in Tasks.

The camera intrinsic fields and 6-DoF pose workflow are hidden from normal
operation. Their backend implementation remains available for future
developer-only experiments, but it does not gate workspace calibration.

## Persistence

Calibration writes median-filtered `reference_points_px`,
`reference_workspace_corners_px`, `reference_resolution`, the detected
dictionary, timestamp, and fit metrics into
`camera.calibration.workspace_aruco` in `robot.local.yaml`.

Moving the camera, changing resolution or zoom, or moving the tags requires a
new calibration and verification.
