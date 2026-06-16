# Remaining Implementation Plan

This document replaces the earlier broad UI/control roadmap with a more honest remaining-work plan based on:

- The original UI/control improvement prompt.
- The previous roadmap and the partial implementation already in the repo.
- The new notes in `comments.md`.
- A quick review of the current PC app, firmware protocol, config, and docs.

The current idea is to implement this in small packages, not as one large rewrite. Each package has a code so later requests can be specific, for example: `Implement UI-11` or `Implement ENC-10`.

## Current Reality

Some roadmap items now exist only as rough plumbing or placeholders. They should not be treated as complete.

Currently present:

- The main tabs are renamed to `Control`, `Tasks`, `Kinematics`, `Program`, and `Settings`.
- The old permanent COM text box has been replaced with a serial modal.
- The Control tab has a tool selector for gripper vs magnet.
- The backend has a `tools` config section and generic `TOOL` commands.
- The backend has a Standard DH data model and a Jacobian IK implementation.
- The 3D view now has DH-based rendering and object marker plumbing.
- The backend has basic color/blob vision helpers and task sequence builders.
- The firmware protocol parses/prints some newer status fields.
- The backend parses encoder readback fields and has staged encoder state.

Still lacking or not production-ready:

- Tool UI and tool settings are not complete enough for real hardware setup.
- Tool dimensions do not properly drive the active TCP model yet.
- Camera preview is still in the side panel instead of a movable popup.
- DH editing is not a real professional table/editor.
- Encoder and end-effector pin settings are not editable in the UI.
- The Settings UI incorrectly suggests elbow/wrist encoders as active concepts, even though those joints are servos.
- AS5048A config is not truly driven from the PC app; firmware still uses compile-time SPI/CS defaults.
- Calibration is still mostly a config dump, not a guided workflow.
- Task and vision flows are placeholders, not a real operator workflow.
- Motion planning and speed/acceleration tuning need serious review before real robot demos.

## Recommended Implementation Order

1. `UI-10`: Fix tool panel visibility and command state polish.
2. `UI-11`: Move camera preview into a movable popup.
3. `UI-12`: Fix HUD/fader placement and collapse arrows.
4. `KIN-10`: Replace DH parameter inputs with a real table editor.
5. `SET-10`: Add end-effector dimensions and IO editor.
6. `ENC-10`: Add base/shoulder encoder settings UI only.
7. `TOOL-11`: Make active tool TCP affect FK, IK, preview, and task poses.
8. `KIN-11`: Harden DH FK/Jacobian IK and diagnostics.
9. `CAL-10`: Add manual arm calibration workflow.
10. `FW-10`: Implement real gripper/magnet IO in firmware.
11. `ENC-11`: Make AS5048A readback config-driven.
12. `ENC-12`: Use encoder readback for known pose and homing.
13. `VISION-10`: Build the live camera/detection workflow properly.
14. `VISION-11`: Add camera-to-robot calibration UI.
15. `TASK-10`: Rebuild task execution UX around preview-first workflows.
16. `TASK-11`: Implement configurable batch color sorting.
17. `CTRL-10`: Improve motion profiles, progress tracking, and tuning.
18. `CAL-11`: Add calibration validation and measured-point checks.
19. `ENC-13`: Add encoder verification and fault detection.
20. `ENC-14`: Add bounded settle correction.
21. `ENC-15`: Defer full closed-loop stepper control until the earlier encoder stages work on hardware.

## Immediate UI Corrections

### UI-10: Tool Panel Visibility And State

Problem:

- The selected tool should be the only tool with visible controls.
- The current tool UI is shallow and can still feel like a demo placeholder.

Work:

- Make gripper controls visible only for `gripper`.
- Make magnet controls visible only for `magnet`.
- Show clear tool state: selected tool, command state, last command result.
- Disable irrelevant buttons when disconnected, not armed, or unsupported by the selected tool.
- Keep the slider only for tools that support proportional values.

Acceptance:

- Selecting `Gripper` shows only open, close, and slider/set controls.
- Selecting `Magnet` shows only on/off controls.
- No hidden or irrelevant gripper buttons remain in the magnet workflow.
- UI state updates immediately and stays synced after backend responses.

### UI-11: Movable Camera Popup

Problem:

- The camera preview is too large for the side panel and makes the Tasks tab cramped.

Work:

- Remove the large inline camera preview from the side panel.
- Add a `View Camera` button in Tasks.
- Open a movable, resizable camera popup over the viewport.
- Keep camera annotations in the popup.
- Allow closing/minimizing the popup without losing detections.

Acceptance:

- Tasks side panel stays compact.
- Camera can be opened, moved, resized, and closed.
- Detection list and task preview still work when the popup is closed.

### UI-12: HUD And Fader Chrome Polish

Problem:

- HUD/fader placement is awkward.
- Collapse arrows still do not communicate the correct direction well enough.

Work:

- Reposition HUD and fader widgets so they do not fight the rail, left panel, or viewport content.
- Replace text arrow glyphs with clear icon buttons.
- Make collapsed state visually obvious.
- Test desktop and smaller viewport sizes.

Acceptance:

- HUD/fader widgets do not overlap important controls.
- Closed/open arrows point in the correct direction.
- The collapsed controls are still usable and visually clear.

## Settings And IO

### SET-10: End-Effector Dimensions And IO Editor

Problem:

- Tool settings exist in config but cannot be properly edited in the UI.
- End-effector pins are not configurable from Settings.
- Gripper and magnet need different physical dimensions and IO.

Work:

- Add editable tool presets in Settings:
  - tool type,
  - display label,
  - TCP offset x/y/z,
  - gripper PWM pin,
  - gripper pulse min/max,
  - gripper open/closed values,
  - magnet GPIO pin,
  - magnet active polarity.
- Save all values to `robot.local.yaml`.
- Validate pins and ranges before saving.

Acceptance:

- User can edit gripper dimensions and servo IO.
- User can edit magnet dimensions and GPIO IO.
- Invalid pins/ranges are shown clearly and do not silently save.

### ENC-10: Base/Shoulder Encoder Settings UI

Problem:

- Settings currently presents encoder data in a summary, not an editable setup.
- Elbow and wrist should not be shown as encoder axes for now because they are normal servos.

Work:

- Add editable encoder settings for base and shoulder only:
  - enabled,
  - CS pin,
  - zero offset,
  - direction sign,
  - readback tolerance,
  - fault tolerance.
- Remove elbow/wrist encoder rows from the normal settings UI.
- Keep internal config flexible enough to add more encoder axes later.

Acceptance:

- UI only shows base and shoulder AS5048A setup.
- User can save encoder CS pins and calibration values.
- Elbow/wrist are clearly treated as servos, not encoder axes.

## Kinematics And Calibration

### KIN-10: Professional DH Table Editor

Problem:

- DH parameters are currently rendered as generic input blocks, not a real table.
- Units and meaning are not clear enough.

Work:

- Replace the current DH editor with a proper table.
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
- Add a visible save/apply control for the DH table.
- Show FK result from the current DH table before saving.

Acceptance:

- DH editor looks and behaves like a table.
- Bad numeric values are rejected visibly.
- Saving updates `robot.local.yaml`.
- FK preview updates predictably.

### KIN-11: Harden DH FK And Jacobian IK

Problem:

- The current solver is useful as a first pass, but not trustworthy enough for real demo use.
- Solver tolerances and diagnostics are not integrated cleanly with the configured kinematics settings.

Work:

- Use kinematics config tolerances and damping everywhere.
- Add clear IK diagnostics:
  - success/failure,
  - position error,
  - orientation error,
  - iteration count,
  - limit warnings,
  - singularity warnings.
- Improve seed selection.
- Add workspace and reachability prechecks.
- Add tests for representative robot poses and unreachable targets.

Acceptance:

- Reachable targets converge reliably in tests.
- Unreachable targets fail clearly.
- UI shows why a target failed instead of only saying "IK failed".

### CAL-10: Manual Arm Calibration Workflow

Problem:

- Calibration is currently scattered across Settings fields.
- There is no guided workflow for zeroing, measuring, and validating the robot.

Work:

- Add a Calibration view or guided Settings section.
- Steps:
  - enter measured link dimensions,
  - enter active tool dimensions,
  - set joint zero offsets,
  - set direction signs,
  - set joint limits,
  - save home pose,
  - save safe pose,
  - store movement tolerance.
- Save to `robot.local.yaml`.

Acceptance:

- A user can calibrate the arm without editing YAML manually.
- The app clearly separates measured values from example defaults.
- Calibration status is visible.

### CAL-11: Calibration Validation

Problem:

- There is no workflow for checking whether calibration is actually good.

Work:

- Add validation tools:
  - FK at home pose,
  - named target reachability,
  - joint limit checks,
  - measured point comparison,
  - encoder-vs-commanded comparison when encoders exist.
- Add warnings when tool dimensions change but validation has not been rerun.

Acceptance:

- User can run a calibration validation pass.
- Failures point to likely causes such as tool length, zero offset, or direction sign.

## Tool And Firmware

### TOOL-11: Active Tool TCP Integration

Problem:

- Tool presets exist, but active tool dimensions do not properly drive the robot model.

Work:

- Apply active tool TCP offset in FK.
- Use active tool TCP in IK.
- Use active tool TCP in task pickup/dropoff targets.
- Update 3D view so gripper/magnet length changes the visible tool point.

Acceptance:

- Switching from gripper to magnet changes the TCP used by FK/IK.
- Task targets use the correct tool dimensions.
- The view and backend agree on the TCP position.

### FW-10: Real Tool IO Firmware Support

Problem:

- `TOOL OPEN/CLOSE/SET/ON/OFF` exists at protocol level, but the firmware does not yet drive real gripper or magnet IO from config.

Work:

- Add firmware config for tool IO.
- Drive 180 degree microservo gripper from configured PWM pin and pulse range.
- Drive electromagnet from configured GPIO and polarity.
- Add safe defaults on boot, disconnect, stop, and fault.

Acceptance:

- Gripper commands move the configured servo output.
- Magnet commands switch the configured output.
- Tool output fails safe on stop/fault/reset.

## Encoder Track

### ENC-11: Config-Driven AS5048A Readback

Problem:

- Firmware has AS5048A readback plumbing, but SPI pins and CS pins are not truly controlled from the PC configuration.

Work:

- Decide whether SPI SCK/MISO/MOSI are compile-time or runtime config.
- Make base/shoulder CS pins configurable.
- Apply zero offset and direction sign to reported encoder angles.
- Report raw and calibrated values where useful.

Acceptance:

- UI-configured base/shoulder CS pins are used by firmware.
- UI shows raw/calibrated encoder readback.
- Missing/bad encoders are visible and not trusted silently.

### ENC-12: Encoder Known Pose And Homing

Problem:

- Encoder readback is not yet a proper known-pose or homing workflow.

Work:

- Add `Use Encoder Pose` or `Set Home From Encoders`.
- Mark pose source as `encoder` or `mixed`.
- Allow known pose from valid base/shoulder encoders and manual/servo pose for the rest.
- Store encoder offsets in local config.

Acceptance:

- Robot can establish known pose without manual `SETPOSE` for encoder-backed axes.
- UI clearly shows pose source.

### ENC-13: Encoder Verification And Fault Detection

Problem:

- Encoder errors are computed roughly, but there is no polished verification workflow.

Work:

- After each move, compare commanded vs encoder angle for base/shoulder.
- Add warning and fault thresholds.
- Add diagnostics:
  - target,
  - encoder,
  - error,
  - tolerance,
  - fault state.

Acceptance:

- Step loss or mismatch is detected.
- The app stops trusting pose after large mismatch.

### ENC-14: Stepper Settle Correction

Problem:

- No bounded correction exists after motion.

Work:

- After move completion, compare encoder angle to target.
- If error exceeds settle tolerance, issue a small correction move.
- Limit correction attempts.
- Fault after repeated failure.

Acceptance:

- Small final errors can be corrected.
- Large disagreement becomes a fault, not an endless correction loop.

### ENC-15: Experimental Full Closed-Loop Stepper Control

Status:

- Deferred.

Reason:

- Full closed-loop stepper control needs filtering, wraparound handling, backlash handling, PID or equivalent control, max correction rates, and stall detection.

Acceptance before starting:

- `ENC-11` through `ENC-14` work on real hardware.
- Base/shoulder encoders are mechanically stable and calibrated.

## Vision And Tasks

### VISION-10: Live Camera And Detection Workflow

Problem:

- Current vision is a request/response placeholder. It is not a good operator workflow.

Work:

- Add camera popup from `UI-11`.
- Add refresh/live modes.
- Show annotations, color labels, pixel coordinates, and robot coordinates.
- Keep detection list in Tasks.
- Add clear empty/error states.

Acceptance:

- User can see detections clearly without crowding the side panel.
- Detection list and popup annotations match.

### VISION-11: Camera-To-Robot Calibration UI

Problem:

- The backend has a 4-point transform helper, but there is no UI workflow.

Work:

- Let the user click or enter four image points.
- Let the user enter corresponding robot XY points.
- Save calibration to `robot.local.yaml`.
- Show transformed robot coordinates for detections.

Acceptance:

- Four-point calibration can be completed from the UI.
- Detections include robot coordinates after calibration.

### TASK-10: Task Workflow Rebuild

Problem:

- Tasks are placeholders and do not yet feel like real operator flows.

Work:

- Rebuild Tasks around:
  - task selector,
  - task-specific settings,
  - detection/object selection,
  - preview,
  - confirmation,
  - execute,
  - compact status/progress.
- Keep task logic separate from vision, IK, motion, and firmware.

Acceptance:

- Pick-and-place can be previewed and executed from a clear workflow.
- User can see exactly what object and drop zone will be used.

### TASK-11: Configurable Batch Color Sorting

Problem:

- Batch color sorting exists only as simple sequence generation.

Work:

- Detect all visible objects.
- Group by selected color profiles.
- Map each color to configured drop zone.
- Generate sequence.
- Show object list and preview path.
- Require confirmation before execution.

Acceptance:

- User can choose which colors to sort.
- Sorting is not hardcoded.
- Preview shows full sequence before motion.

### VIEW-10: Object Markers In 3D View

Problem:

- Object marker plumbing exists, but it depends on calibration and is not yet part of a robust operator workflow.

Work:

- Display calibrated detections in the 3D view.
- Keep object markers independent from arm preview/path clearing.
- Use clear colors and labels.
- Add visibility toggle.

Acceptance:

- Detected objects appear in correct robot-frame locations after camera calibration.
- Preview/path clearing does not remove object markers.

## Motion And Control

### CTRL-10: Motion Profiles And Tuning

Problem:

- Movement, speed, and acceleration need a deeper review before demo use.

Work:

- Review current rate limiting and trajectory generation.
- Make synchronized joint arrival explicit.
- Improve speed/acceleration defaults.
- Add clear units and per-joint constraints.
- Add profile preview with estimated duration.

Acceptance:

- Motion preview duration is credible.
- Joints arrive together for joint-space moves.
- Speed/acceleration settings are understandable and bounded.

### CTRL-11: Execution Progress And Abort Behavior

Problem:

- The UI does not yet provide enough confidence during execution.

Work:

- Show active waypoint/step.
- Show progress through path/task.
- Make stop behavior consistent across path, task, live jog, and serial motion.
- Ensure preview state clears only when intended.

Acceptance:

- User can tell what the robot is currently doing.
- Stop reliably aborts task/path/live motion.

### CTRL-12: Workspace And Safety Limits

Problem:

- Joint limits exist, but workspace limits and task-level safety are weak.

Work:

- Add configurable workspace bounds.
- Validate task-generated pickup/dropoff targets.
- Check approach heights and safe pose reachability.
- Block tasks that violate limits before execution.

Acceptance:

- Invalid task targets fail at preview time.
- Safety errors explain what needs to be changed.

## Diagnostics And Tests

### DIAG-10: Diagnostics Drawer Completion

Problem:

- Diagnostics exists but is still a dump rather than a useful debugging tool.

Work:

- Organize diagnostics into sections:
  - serial,
  - motion,
  - IK,
  - encoder,
  - config sync,
  - tool,
  - vision.
- Add clear buttons and export/copy option if useful.

Acceptance:

- Debugging info is available without cluttering normal operation.

### TEST-10: UI Regression Tests

Problem:

- Browser behavior is mostly manually smoke-tested.

Work:

- Add automated UI smoke/regression tests for:
  - preview persistence,
  - live jog stability,
  - tool switching,
  - serial modal,
  - camera popup,
  - DH table save,
  - diagnostics drawer.

Acceptance:

- UI regressions are caught by repeatable tests.

### TEST-11: Hardware/Firmware Protocol Tests

Problem:

- Firmware builds, but protocol behavior is not fully regression-tested.

Work:

- Add protocol tests for:
  - `TOOL ON/OFF`,
  - config while armed but idle,
  - encoder status fields,
  - pose source fields,
  - bad config rejection.

Acceptance:

- Protocol changes can be made without breaking the dashboard.

## Open Questions Before Hardware Work

These should be answered before implementing the hardware-heavy packages:

- What are the exact gripper servo pin, pulse min/max, open value, and close value?
- What are the electromagnet transistor GPIO pin and active polarity?
- What voltage/current does the magnet need, and what is the safe default output state?
- Are AS5048A encoders mounted at the joint output for base and shoulder?
- What SPI pins are actually wired for AS5048A?
- Are encoder zero positions mechanically repeatable?
- What are the real measured gripper and magnet TCP offsets?
- What camera source index, resolution, and physical mounting height will be used?
- What are the demo drop zones and object colors?
- What speed and acceleration values are safe for the printed/mechanical arm?

## Suggested Next Requests

Good next implementation requests:

```text
Implement UI-10 and UI-12 only.
```

```text
Implement KIN-10 only. Do not change IK math.
```

```text
Implement SET-10 and ENC-10, but only the UI/config layer. Do not change firmware yet.
```

```text
Implement FW-10 for the protocol stub and arm controller, with tests/build verification.
```

Avoid requests like:

```text
Implement everything in the remaining plan.
```

The current project will move faster if each package is implemented, tested, and reviewed separately.
