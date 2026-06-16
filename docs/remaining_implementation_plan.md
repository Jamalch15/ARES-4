# Remaining Implementation Plan By Workpiece

This document replaces the earlier scattered roadmap with a subsystem-based plan. The goal is to make it easy to request implementation in useful chunks, for example:

```text
Implement TOOL-01 and TOOL-02.
```

```text
Implement KIN-01 through KIN-03 only.
```

The project is still in an early stage. Treat the structure below as the current working plan, not a fixed final architecture.

## Current Reality

Some earlier roadmap items have been partially implemented, but several are still placeholder-level.

Currently present at a rough level:

- Main tabs have been renamed to `Control`, `Tasks`, `Kinematics`, `Program`, and `Settings`.
- The old permanent COM text box has been replaced with a serial modal.
- The Control tab has a gripper/magnet tool selector.
- The backend has generic `TOOL` commands and a `tools` config section.
- The backend has a Standard DH data model and Jacobian IK plumbing.
- The app has an editable MATLAB prototype geometry preset with `L_1..L_9`, `s4`, `s6`, and `s8`.
- The 3D view has DH-based rendering and object marker plumbing.
- The backend has basic color/blob vision helpers and task sequence builders.
- Firmware/backend protocol parsing includes some newer status fields.
- Encoder readback fields exist in software, but are not a real hardware workflow yet.

Still missing or not production-ready:

- Control preview/live jog behavior needs to be proven stable.
- Tool controls are still shallow and should show only the selected tool controls.
- Tool dimensions do not fully drive the active TCP model.
- Tool and encoder pins cannot be properly edited from Settings.
- Camera preview should move out of the side panel into a movable popup.
- DH editing needs to become a proper table, not generic input blocks.
- The current kinematics implementation should be reconciled with the MATLAB prototype.
- The MATLAB motion prototype has useful ideas, but it lacks motor velocity/acceleration limits for real execution.
- Calibration is not yet a guided workflow.
- Settings should show encoders only for base and shoulder. Elbow and wrist are servos for now.
- AS5048A encoder configuration is not truly PC-app driven yet.
- Vision and task workflows are placeholders, not operator-ready workflows.
- Diagnostics and tests need to be organized around real failure modes.

## MATLAB Prototype Summary

Prototype reviewed: `jacobian_ik_robotarm_analytic_seed.m`.

The MATLAB file implements a useful working prototype for robot geometry, DH forward kinematics, analytic IK seeding, Jacobian movement, visualization, and motion diagnostics.

### Physical Model In The Prototype

Working assumption: these are current measured/prototype values, not final calibration values.

```text
L_1 = 93.45 mm
L_2 = 23.20 mm
L_3 = 64.50 mm
L_4 = 42.69 mm
L_5 = 160.15 mm
L_6 = 41.39 mm
L_7 = 142.55 mm
L_8 = 49.20 mm
L_9 = 15.00 mm

s4 = -1
s6 = -1
s8 =  1
```

Joint limits in the prototype:

```text
theta1: -180 to 180 deg
theta2:  -90 to 160 deg
theta3: -160 to 160 deg
theta4: -180 to 180 deg
```

Starting pose:

```text
[0, 0, -70, -20] deg
```

Movement tuning in the prototype:

```text
end-effector speed: 25 mm/s
dt: 0.02 s
max move time: 60 s
position tolerance: 1.0 mm
phi tolerance: 2 deg
Jacobian damping lambda: 0.1
phi convergence gain: 1.5
prefer elbow down: true
```

### DH And FK Implemented In The Prototype

The prototype uses Standard DH. The active DH table is:

```text
joint  theta   alpha   a     d          extra measured offset
1      th1     pi/2    0     L1 + L3   L2 side offset after d1
2      th2     0       L5    s4 * L4
3      th3     0       L7    s6 * L6
4      th4     0       L9    s8 * L8
```

It computes:

- Full transform chain.
- Joint points.
- End-effector position.
- Pitch-like tool angle `phi = theta2 + theta3 + theta4`.
- Linear Jacobian from DH frame axes using cross products.
- Full Jacobian including angular rows.
- Segment data for visualizing DH `d` and `a` offsets separately.

### Analytic Seed IK Implemented In The Prototype

The prototype does not rely on Jacobian IK from a random seed. It first computes an analytic seed using the robot geometry.

Main ideas:

- Combine lateral offsets into `B = s4*L4 + s6*L6 + s8*L8`.
- Solve base angle from the XY target while accounting for the lateral offset.
- Solve the shoulder/elbow planar 2-link part using law of cosines.
- Compute wrist angle from `theta4 = phiTarget - theta2 - theta3`.
- Try multiple candidate branches.
- Reject candidates outside joint limits.
- Score candidates using:
  - FK position error,
  - phi error,
  - elbow-down preference,
  - continuity from the current pose.

This is important. It gives the numerical Jacobian solver a realistic starting point and reduces weird solutions near joint limits.

### Jacobian Movement Implemented In The Prototype

The movement function simulates Cartesian end-effector velocity control.

At each time step it:

- Computes current FK and Jacobian.
- Computes position error and phi error.
- Commands a linear end-effector velocity toward the target.
- Commands phi convergence with a bounded phi rate.
- Builds a task Jacobian:

```text
J_task = [Jv; J_phi]
J_phi = [0, 1, 1, 1]
```

- Solves damped least squares:

```text
thetaDot = J_task' * inv(J_task * J_task' + lambda^2 * I) * xDotTask
```

- Adds a weak attraction toward the analytic seed.
- Wraps and clamps joint angles to limits.
- Stops when position and phi tolerances are reached.

Important limitation: the function name includes `NoMotorLimit`. It does not properly enforce real motor velocity, acceleration, step timing, or synchronized arrival constraints. It is good as a preview/simulation model, but it should not be treated as real execution control until those constraints are added.

### Visualization And Diagnostics Implemented In The Prototype

The prototype includes:

- 3D animation of the arm path.
- Separate drawing of DH `d` and `a` segments.
- End-effector path trace.
- Target marker.
- Phi direction arrow.
- Plots for desired vs actual end-effector speed.
- Joint velocity plots.
- Position error and phi error plots.
- Warnings when final joints are near limits.

## What From The MATLAB Prototype Belongs In This Roadmap

These ideas should be added to the implementation plan:

- Use the measured `L_1..L_9` and sign-offset model as a geometry preset or calibration starting point.
- Make measured geometry the editable source of truth, with Standard DH derived from the MATLAB dimensions as the first known-good physical model.
- Add an analytic seed step before numerical Jacobian IK.
- Use DH frame axes to compute the Jacobian, not ad hoc or inconsistent derivatives.
- Preserve the `phi = theta2 + theta3 + theta4` task orientation model for the first pass.
- Add elbow-down and continuity preferences to IK solution scoring.
- Add Cartesian velocity preview using damped least squares.
- Add movement diagnostics: estimated duration, EE speed, joint velocities, position error, phi error, and limit warnings.
- Add DH segment visualization to the 3D view so measured geometry is understandable.
- Add tests that compare Python FK/IK behavior against known MATLAB prototype targets.

These ideas should not be ported directly:

- The MATLAB command-window loop.
- MATLAB figures as the app UI model.
- The no-motor-limit movement loop as real robot execution.
- Any assumption that the prototype dimensions are final calibration.
- Any full closed-loop control behavior.

## Recommended Implementation Order

This order keeps useful operator fixes first, then builds the kinematics and movement foundation before adding heavier vision/task behavior.

1. `SHELL-01` through `SHELL-04`: fix the operator shell, preview stability, camera popup, HUD/fader, and serial modal polish.
2. `TOOL-01` through `TOOL-04`: finish end-effector selection, settings, active TCP, and firmware IO behavior.
3. `KIN-01` through `KIN-06`: reconcile the app with the MATLAB DH/IK model.
4. `MOVE-01` through `MOVE-05`: build movement preview, constraints, progress, abort, and diagnostics.
5. `CAL-01` through `CAL-04`: add guided arm/tool calibration and validation.
6. `ENC-01` through `ENC-05`: implement AS5048A readback, known pose, verification, and bounded correction.
7. `VISION-01` through `VISION-04`: build the camera popup, detections, profiles, and camera-to-robot calibration.
8. `TASK-01` through `TASK-04`: rebuild task workflows around preview-first operation.
9. `VIEW-01` through `VIEW-03`: polish 3D visualization, object markers, and DH segment overlays.
10. `DIAG-01`, `TEST-01`, `TEST-02`, and `TEST-03`: harden diagnostics and regression tests.

## Workpiece: Operator Shell And Control UI

### SHELL-01: Control Preview And Live Jog Stability

Status: needs verification and likely refinement.

Work:

- Keep preview visible after pressing Apply.
- Clear preview only when target is reached, Reset is pressed, or a newer target replaces it.
- Separate frontend state into:
  - draft target,
  - commanded target,
  - reported pose,
  - target reached.
- Stop websocket pose updates from recreating or clearing the preview during live jog.
- Stabilize Apply and Reset buttons so state changes do not shift layout.

Acceptance:

- Apply does not immediately remove the preview.
- Live jog preview does not twitch.
- Apply/Reset buttons do not visually twitch.

### SHELL-02: Camera Popup Instead Of Side-Panel Preview

Status: missing.

Work:

- Remove the large inline camera preview from the Tasks side panel.
- Add a `View Camera` button.
- Open a movable, resizable camera popup over the viewport.
- Keep annotations in the popup.
- Let detections continue when the popup is closed or minimized.

Acceptance:

- Tasks panel stays compact.
- Camera can be opened, moved, resized, and closed.
- Detection state remains usable without the popup open.

### SHELL-03: HUD And Fader Chrome Polish

Status: missing.

Work:

- Reposition HUD and fader widgets so they do not fight the rail, left panel, or viewport.
- Fix collapse arrow direction.
- Prefer clear icon buttons instead of ambiguous text arrows.
- Verify desktop and smaller viewport layouts.

Acceptance:

- HUD/fader controls do not overlap important controls.
- Closed/open arrows point correctly.
- Collapsed controls remain usable.

### SHELL-04: Serial Modal Polish

Status: partial.

Work:

- Keep the serial picker as a modal, not a permanent large widget.
- Show available COM ports with descriptions.
- Keep baud rate secondary but configurable.
- Save last selected port to local config.
- Handle unavailable/stale ports clearly.

Acceptance:

- User can connect without typing COM names manually.
- Stale/disconnected ports fail clearly.

## Workpiece: End Effector And Tooling

### TOOL-01: Selected Tool Controls Only

Status: partial.

Work:

- Show gripper controls only when active tool is `gripper`.
- Show magnet controls only when active tool is `magnet`.
- Gripper controls:
  - open,
  - close,
  - proportional slider `0.0..1.0`.
- Magnet controls:
  - on,
  - off.
- Disable irrelevant commands when disconnected, unsupported, or unsafe.

Acceptance:

- Selecting `Gripper` shows only gripper controls.
- Selecting `Magnet` shows only magnet controls.
- Tool UI state follows backend state.

### TOOL-02: End-Effector Settings And IO Editor

Status: missing.

Work:

- Add editable tool presets in Settings:
  - tool type,
  - display name,
  - TCP offset x/y/z,
  - gripper PWM pin,
  - gripper pulse min/max,
  - gripper open/closed values,
  - magnet GPIO pin,
  - magnet active polarity.
- Save values to `robot.local.yaml`.
- Validate pins, ranges, and units before saving.

Acceptance:

- Gripper dimensions and servo IO can be edited from the UI.
- Magnet dimensions and GPIO IO can be edited from the UI.
- Invalid values do not silently save.

### TOOL-03: Active Tool TCP Integration

Status: partial or missing.

Work:

- Apply the active tool TCP offset in FK.
- Use active tool TCP in IK.
- Use active tool TCP in pickup/dropoff task targets.
- Update the 3D view so gripper/magnet length changes the visible TCP.
- Mark calibration stale when active tool geometry changes.

Acceptance:

- Switching tool type changes the TCP used by FK/IK.
- Task targets use the selected tool geometry.
- Backend and 3D view report the same TCP position.

### TOOL-04: Real Firmware Tool IO

Status: protocol exists, real IO likely incomplete.

Work:

- Implement `TOOL OPEN`, `TOOL CLOSE`, and `TOOL SET value=...` for the gripper.
- Implement `TOOL ON` and `TOOL OFF` for the magnet.
- Drive a 180 degree microservo from configured PWM and pulse range.
- Drive electromagnet output from configured GPIO and polarity.
- Set safe tool output on boot, stop, fault, and disconnect.

Acceptance:

- Gripper commands move the configured servo output.
- Magnet commands switch the configured output.
- Tool outputs fail safe on stop/fault/reset.

## Workpiece: Robot Geometry, DH, FK, And IK

### KIN-01: Import MATLAB Physical Geometry As A Preset

Status: implemented as a first pass.

Reality note: the MATLAB values are stored as a `working_assumption` preset, not as final calibration. Applying the preset fills a DH draft; it does not silently prove the physical arm is calibrated.

Work:

- Add a named geometry preset based on the MATLAB prototype.
- Store values as measured link dimensions, not hardcoded solver constants.
- Include `L_1..L_9` and sign values `s4`, `s6`, `s8`.
- Keep `L_2` in the active measured model as a base side offset, not as
  first-row `a1`.
- Label all units as `mm` and `deg`.

Acceptance:

- The app can load the MATLAB prototype geometry as a starting model.
- The user can see and edit measured dimensions before applying them.

### KIN-02: Professional DH Table Editor

Status: implemented as a first pass.

Reality note: the table editor now validates and previews DH drafts, but it is not yet a guided calibration workflow.

Work:

- Replace generic DH inputs with a table editor.
- Columns:
  - joint,
  - theta offset deg,
  - d mm,
  - a mm,
  - alpha deg,
  - min deg,
  - max deg,
  - zero offset deg,
  - direction sign.
- Add row-level validation.
- Show FK preview before saving.
- Save to `robot.local.yaml`.

Acceptance:

- Measured geometry is edited in one place.
- Derived DH values are shown read-only and validated visibly.
- FK preview updates predictably from the derived model.

### KIN-03: DH Forward Kinematics Aligned With MATLAB

Status: implemented as a first pass.

Reality note: Python FK is tested against selected measured-prototype DH poses. The coordinate-frame convention is still a project working assumption until measured against the physical arm.

Work:

- Derive Standard DH from the active measured-geometry preset.
- Verify the app uses the same transform order as the MATLAB prototype.
- Support the measured-prototype table:

```text
d1 = L1 + L3
base side offset = L2
d2 = s4 * L4
d3 = s6 * L6
d4 = s8 * L8
a2 = L5
a3 = L7
a4 = L9
alpha1 = 90 deg
alpha2..4 = 0 deg
```

- Apply active tool TCP after the final joint transform.
- Add FK tests against known measured-prototype poses.

Acceptance:

- Python FK matches the MATLAB model for selected test poses.
- The 3D view and backend agree on joint frames and TCP position.

### KIN-04: Analytic Seed Before Jacobian IK

Status: missing or incomplete.

Work:

- Port the MATLAB analytic seed concept into backend kinematics.
- Use lateral offset `B = d2 + d3 + d4`.
- Solve base angle while accounting for offset.
- Solve shoulder/elbow with 2-link planar IK.
- Compute wrist angle from target phi.
- Generate candidate branches.
- Reject candidates outside joint limits.
- Score with:
  - FK position error,
  - phi error,
  - elbow-down preference,
  - continuity from current pose.

Acceptance:

- Numerical IK starts from a realistic seed.
- Common targets converge more reliably.
- Solver chooses less surprising joint configurations.

### KIN-05: DH-Based Jacobian IK Diagnostics

Status: partial.

Work:

- Compute linear Jacobian from DH frame axes using cross products.
- Use damped least squares.
- Keep the first-pass orientation task as `phi = theta2 + theta3 + theta4`.
- Report:
  - success/failure,
  - final position error,
  - final phi/orientation error,
  - iteration count,
  - limit warnings,
  - singularity or near-singularity warnings,
  - selected seed source.

Acceptance:

- Reachable targets converge in tests.
- Unreachable targets fail clearly.
- The UI explains why IK failed.

### KIN-06: Workspace And Reachability Checks

Status: missing.

Work:

- Add a fast reachability precheck before expensive IK.
- Check joint limits, workspace bounds, and target approach height.
- Show whether target failure is due to reach, joint limits, or invalid orientation.

Acceptance:

- Invalid targets fail before execution.
- Operator sees a useful reason, not just `IK failed`.

## Workpiece: Movement, Trajectory, And Execution

### MOVE-01: Motion State And Preview Reliability

Status: partial.

Work:

- Keep draft, commanded, reported, and reached states separate.
- Ensure preview/path/object markers are independent layers.
- Clear preview only from explicit state transitions.
- Keep Home and Stop available in the main view.

Acceptance:

- Preview behavior is stable across Apply, Reset, live jog, websocket updates, and task execution.

### MOVE-02: MATLAB-Style Cartesian Velocity Preview

Status: missing.

Work:

- Add a preview-only movement simulation based on the MATLAB loop:
  - fixed end-effector speed,
  - fixed `dt`,
  - damped least squares,
  - phi convergence,
  - analytic seed attraction,
  - joint wrapping and clamping.
- Show estimated path, duration, speed, and final error.
- Keep this marked as preview/simulation until motor limits are added.

Acceptance:

- User can preview a smooth Cartesian move before execution.
- Preview reports estimated duration and final error.
- Preview does not pretend to be a guaranteed hardware execution path.

### MOVE-03: Real Joint Limits, Velocity Limits, And Acceleration Limits

Status: missing.

Work:

- Add per-joint velocity and acceleration limits.
- Add stepper/servo command-rate constraints.
- Add synchronized joint arrival for joint-space moves.
- Add bounded Cartesian preview that respects motor limits.
- Make units explicit.

Acceptance:

- Generated moves respect configured joint limits and speed limits.
- Estimated duration is credible for real hardware.

### MOVE-04: Execution Progress And Abort Behavior

Status: partial.

Work:

- Show active move, waypoint, or task step.
- Show progress through generated path.
- Make Stop behavior consistent across:
  - direct joint movement,
  - live jog,
  - path execution,
  - tasks,
  - serial motion.
- Keep internal emergency/fault handling, but do not expose a large E-stop UI unless needed.

Acceptance:

- User can tell what the robot is currently doing.
- Stop reliably aborts motion/task/live jog.

### MOVE-05: Motion Diagnostics

Status: missing.

Work:

- Add diagnostics inspired by MATLAB plots:
  - desired vs actual EE speed,
  - joint velocity history,
  - position error,
  - phi error,
  - near-limit warnings,
  - estimated vs actual move duration.
- Keep this in Diagnostics, not normal operator UI.

Acceptance:

- Movement behavior can be debugged after a bad move without cluttering normal operation.

## Workpiece: Arm And Tool Calibration

### CAL-01: Manual Arm Geometry Calibration

Status: missing.

Work:

- Add a guided workflow for:
  - measured link dimensions,
  - DH rows,
  - joint zero offsets,
  - direction signs,
  - joint limits,
  - home pose,
  - safe pose,
  - movement tolerance.
- Save to `robot.local.yaml`.
- Treat MATLAB dimensions as an editable starting preset.

Acceptance:

- User can calibrate without editing YAML manually.
- App clearly separates example defaults, measured local values, and active values.

### CAL-02: Tool Calibration

Status: missing.

Work:

- Calibrate gripper TCP dimensions.
- Calibrate magnet TCP dimensions.
- Track which tool was active during calibration.
- Warn when tool dimensions change after calibration.

Acceptance:

- Tool geometry is not mixed accidentally between gripper and magnet.
- Calibration status is visible per tool.

### CAL-03: Calibration Validation

Status: missing.

Work:

- Validate FK at home pose.
- Validate named target reachability.
- Compare measured points against FK.
- Show likely causes when validation fails:
  - wrong tool length,
  - wrong zero offset,
  - wrong direction sign,
  - bad DH dimension.

Acceptance:

- User can run a calibration validation pass.
- Failures point to plausible causes.

### CAL-04: Named Positions

Status: partial or missing.

Work:

- Add editable named positions:
  - home,
  - safe,
  - pickup test,
  - dropoff A,
  - dropoff B,
  - user-defined saved positions.
- Validate named positions against joint limits and IK reachability.
- Store in `robot.local.yaml`.

Acceptance:

- Named positions are usable in manual control and tasks.
- Invalid saved poses are rejected or clearly marked.

## Workpiece: Encoders And Known Pose

### ENC-01: Base/Shoulder Encoder Settings UI

Status: missing.

Work:

- Show only base and shoulder encoder setup in normal Settings.
- Do not show elbow/wrist encoders as active concepts for now.
- Add editable values:
  - enabled,
  - CS pin,
  - zero offset,
  - direction sign,
  - readback tolerance,
  - fault tolerance.

Acceptance:

- Settings matches the actual hardware plan.
- User can save base/shoulder encoder settings.

### ENC-02: Config-Driven AS5048A Readback

Status: partial.

Work:

- Decide which SPI pins are compile-time versus runtime config.
- Make base/shoulder CS pins configurable.
- Read AS5048A values over SPI.
- Report raw and calibrated values.
- Apply zero offset and direction sign.
- Report valid/error state.

Protocol fields:

```text
enc=1100
e1=<deg>
e2=<deg>
```

Acceptance:

- UI shows live base/shoulder encoder angles.
- Bad/missing encoder is visible and not trusted silently.

### ENC-03: Encoder Known Pose And Homing

Status: missing.

Work:

- Add `Use Encoder Pose` or `Set Home From Encoders`.
- Mark pose source as:
  - manual,
  - setpose,
  - encoder,
  - mixed.
- Use encoders to establish known pose for base/shoulder.
- Store offsets in local config.

Acceptance:

- Robot can enter known-pose state from valid encoder readback.
- UI clearly shows pose source.

### ENC-04: Encoder Verification And Fault Detection

Status: missing.

Work:

- After motion, compare commanded angle vs encoder angle.
- Warn above tolerance.
- Fault above hard threshold.
- Stop trusting pose after large mismatch.
- Add diagnostics:
  - target,
  - encoder,
  - error,
  - tolerance,
  - fault state.

Acceptance:

- Step loss or mismatch is detected.
- Robot does not continue pretending pose is accurate after large mismatch.

### ENC-05: Bounded Stepper Settle Correction

Status: deferred until readback and verification work on hardware.

Work:

- After move completion, compare encoder angle to target.
- Issue small correction move if error exceeds settle tolerance.
- Limit correction attempts.
- Fault after repeated failure.

Acceptance:

- Small final errors can be corrected.
- Large disagreement becomes a fault, not endless correction.

### ENC-06: Experimental Full Closed-Loop Stepper Control

Status: explicitly deferred.

Do not start this until `ENC-01` through `ENC-05` work on real hardware.

Future work:

- Real-time correction during motion.
- Wraparound handling.
- Backlash handling.
- Filtering.
- PID or equivalent control.
- Max correction rate.
- Stall detection.
- Explicit experimental config flag.

## Workpiece: Vision And Camera

### VISION-01: Live Annotated Camera Popup

Status: missing.

Work:

- Use the movable popup from `SHELL-02`.
- Show live USB camera feed.
- Overlay detected blobs.
- Show:
  - color label,
  - image coordinate,
  - calibrated robot coordinate when available.
- Add refresh/live mode.

Acceptance:

- Camera view is useful without crowding the side panel.
- Annotation and detection list match.

### VISION-02: Color Profile Editor

Status: partial or missing.

Work:

- Add editable color profiles.
- Use HSV thresholds for first pass.
- Save profiles to `robot.local.yaml`.
- Support enabling/disabling colors per task.
- Add minimum blob area and filtering settings.

Acceptance:

- Sorting colors are configurable from UI.
- Detection is not hardcoded.

### VISION-03: Camera-To-Robot Calibration

Status: backend helper exists, UI missing.

Work:

- Add 4-point planar calibration workflow.
- User clicks or enters 4 image points.
- User enters corresponding robot XY points.
- Save transform to `robot.local.yaml`.
- Show transformed robot coordinates for detections.

Acceptance:

- Detections can be mapped to robot-frame coordinates.
- Calibration can be completed from the UI.

### VISION-04: Detection State Contract

Status: missing.

Work:

- Define a clean detection object format:
  - id,
  - label,
  - confidence or quality score,
  - image x/y,
  - robot x/y/z when calibrated,
  - area,
  - timestamp.
- Keep vision output independent from task logic.

Acceptance:

- Task code consumes detections without depending on OpenCV internals.

## Workpiece: Task Workflow

### TASK-01: Dedicated Task Panel

Status: placeholder-level.

Work:

- Rebuild Tasks around:
  - task selector,
  - task-specific settings,
  - camera button,
  - detection list,
  - preview,
  - confirmation,
  - execute,
  - compact status/progress.
- Keep task logic separate from vision, IK, motion execution, and firmware transport.

Acceptance:

- Operator can choose a task and understand what will happen before execution.

### TASK-02: Pick-And-Place Template

Status: partial.

Work:

- Generate reusable sequence:
  - safe pose,
  - move above object,
  - descend,
  - close/enable tool,
  - lift,
  - move above drop zone,
  - descend,
  - open/disable tool,
  - lift,
  - return safe.
- Validate approach height, tool type, and target reachability.

Acceptance:

- Pick-and-place can be previewed and executed from a clear workflow.
- Invalid targets fail at preview time.

### TASK-03: Batch Color Sorting

Status: placeholder-level.

Work:

- Detect all visible colored objects.
- Group by selected color profiles.
- Map each color to a configured drop zone.
- Generate full pick-and-place sequence.
- Show object list and preview path.
- Require confirmation before execution.

Acceptance:

- User can choose which colors to sort.
- Sorting is configurable, not hardcoded.
- Full batch sequence is visible before motion.

### TASK-04: Task Abort And Recovery

Status: missing.

Work:

- Define what happens when a task stops mid-sequence.
- Preserve enough state to tell the user:
  - current step,
  - last completed step,
  - whether tool is holding an object,
  - safe recovery options.

Acceptance:

- Stopping a task does not leave the UI confused about robot/tool state.

## Workpiece: 3D View And Spatial Feedback

### VIEW-01: DH Segment Visualization

Status: implemented as a first pass.

Reality note: the 3D view now renders Standard DH `d` and `a` translations as
separate visible segments and can show labels/frame axes through the Frames
toggle. `L_2` is part of the active measured base side offset and is also shown
in the measured base support sketch.

Work:

- Use measured-prototype segment data to show DH `d` and `a` offsets.
- Add optional labels in diagnostics/calibration mode.
- Keep normal operator view less cluttered.

Acceptance:

- Calibration/debug mode makes the robot geometry understandable.
- Operator mode remains clean.

### VIEW-02: Object Markers In Robot Frame

Status: plumbing exists, workflow incomplete.

Work:

- Display calibrated detections in the 3D robot view.
- Use colored markers and labels.
- Keep object markers independent from preview/path clearing.
- Add a visibility toggle.

Acceptance:

- Detected objects appear in correct robot-frame locations after camera calibration.
- Preview clearing does not remove object markers.

### VIEW-03: Path And Target Layers

Status: partial.

Work:

- Separate layers for:
  - current arm,
  - draft preview,
  - commanded target,
  - task path,
  - object detections,
  - calibration markers.

Acceptance:

- Updating one visual layer does not accidentally erase unrelated information.

## Workpiece: Firmware And Protocol

### FW-01: TB6600-Oriented Hardware Config

Status: partial.

Work:

- Remove software microstep pin fields from the UI.
- Keep microstep value in config for steps-per-degree math.
- Default driver model to `TB6600`.
- Allow config sync while armed only when:
  - controller is idle,
  - no path is running,
  - live jog is off,
  - no task is executing.
- Firmware must reject config sync during motion.

Acceptance:

- Settings match TB6600 physical DIP switch reality.
- Syncing config while armed is possible when idle, but blocked during motion.

### FW-02: Protocol Status Extensions

Status: partial.

Work:

- Keep old firmware compatibility.
- Support optional status fields:

```text
known=0|1
pose_source=<manual|setpose|encoder|mixed|unknown>
enc=1100
e1=<deg>
e2=<deg>
tool=<open|closed|moving|on|off|unknown>
```

Acceptance:

- Old status lines still parse.
- New status lines expose known pose, encoder state, and tool state.

### FW-03: Safe Stop And Fault Semantics

Status: partial.

Work:

- Keep internal emergency/fault handling.
- Use one normal visible Stop button in the UI unless later testing proves a visible E-stop is needed.
- Define firmware responses for stop, fault, clear fault, and reset.
- Ensure tool outputs go safe on stop/fault.

Acceptance:

- Stop behavior is understandable and consistent.
- Safety internals remain available even if the UI is simpler.

## Workpiece: Diagnostics And Tests

### DIAG-01: Hidden Diagnostics Drawer

Status: partial.

Work:

- Keep event logs out of normal Tasks UI.
- Organize diagnostics into:
  - serial,
  - motion,
  - IK,
  - encoder,
  - config sync,
  - tool,
  - vision.
- Add copy/export if useful.

Acceptance:

- Debug information is available without cluttering normal operation.

### TEST-01: Backend Unit Tests

Status: partial.

Work:

- Test config precedence and saving to `robot.local.yaml`.
- Test tool config validation.
- Test DH config load/save.
- Test FK against measured-prototype known poses.
- Test analytic seed candidate behavior.
- Test Jacobian IK success/failure.
- Test encoder parsing.
- Test armed idle-only config sync.

Acceptance:

- Core math/config/protocol behavior is covered by repeatable tests.

### TEST-02: UI Regression Tests

Status: missing.

Work:

- Test:
  - preview persistence,
  - live jog stability,
  - tool switching,
  - serial modal,
  - camera popup,
  - DH table save,
  - diagnostics drawer.

Acceptance:

- Common UI regressions are caught automatically.

### TEST-03: Firmware Build And Protocol Tests

Status: partial.

Work:

- Build protocol stub and controller firmware.
- Test:
  - `TOOL ON/OFF`,
  - `TOOL OPEN/CLOSE/SET`,
  - encoder status fields,
  - config while armed but idle,
  - config rejection during motion,
  - safe tool state on stop/fault.

Acceptance:

- Firmware/protocol changes do not silently break the dashboard.

## Open Questions Before Hardware-Heavy Work

These should be answered before implementing hardware-heavy packages.

- Are the MATLAB `L_1..L_9` values the newest measurements?
- Is the current `L_2` sign/direction correct after measuring against the physical arm?
- Are `s4 = -1`, `s6 = -1`, and `s8 = 1` final for the current build?
- Does the app coordinate frame match the MATLAB coordinate frame?
- Are the MATLAB joint limits final and mechanically safe?
- Should elbow-down be the default for all demo tasks?
- What are the gripper servo pin, pulse min/max, open value, and close value?
- What are the electromagnet transistor GPIO pin and active polarity?
- What is the safe default magnet output state?
- What are the real gripper and magnet TCP offsets?
- Are AS5048A encoders mounted at the base and shoulder joint outputs?
- What SPI pins and CS pins are actually wired?
- Are encoder zero positions mechanically repeatable?
- What camera source index, resolution, and mount height will be used?
- What are the first demo object colors and drop zones?
- What speed and acceleration values are safe for the physical arm?

## Suggested Next Implementation Requests

Good focused requests:

```text
Implement SHELL-01 through SHELL-03 only.
```

```text
Implement TOOL-01 and TOOL-02 only. Do not change firmware yet.
```

```text
Implement KIN-01 through KIN-03 and add tests against the MATLAB geometry.
```

```text
Implement KIN-04 and KIN-05 using the MATLAB analytic seed and Jacobian ideas.
```

```text
Implement MOVE-02 as preview-only. Do not use it for hardware execution yet.
```

Avoid broad requests like:

```text
Implement the whole remaining plan.
```

The project will move faster if each workpiece is implemented, tested, and reviewed separately.
