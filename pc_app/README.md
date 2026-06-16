# Modular 4DOF Arm PC Control App

This is a local control dashboard for the provisional 4DOF robot arm architecture.

Current scope:

- Python backend with FastAPI and WebSockets
- Browser dashboard on localhost
- Manual Control tab for four rotary joints and active tool controls
- Standard DH forward kinematics and Jacobian IK sandbox
- Cartesian target preview with ghost arm, target marker, and path line
- Task panel with camera/detection placeholders and pick/place or color-sorting preview
- In-memory program sandbox for multi-waypoint experiments
- Simulation mode by default
- Serial transport abstraction for later ESP32-S3 control
- Persistent Hardware IO settings for pins, TB6600 microstep value, gear ratios, servo pulse mapping, tools, and encoders
- ESP hardware-config sync on serial connect and Settings save
- Safety checks for joint limits, known pose, stop, armed hardware mode, live-motion gating, and rate-limited motion
- Named-position, task, tool, vision, diagnostics, and encoder-readback APIs for the demo path

Not included in this iteration:

- Real homing switches
- Full closed-loop encoder correction

## Working Assumptions

These are assumptions, not final design decisions.

- Joint 1, base: stepper motor
- Joint 2, shoulder: stepper motor
- Joint 3, elbow: servo motor
- Joint 4, wrist: servo motor
- Initial PC-to-controller transport: USB serial
- Bluetooth may be added later as another transport implementation
- Exact link lengths, gear ratios, zero offsets, pin assignments, servo pulse ranges, and homing hardware are not decided
- Hardware feedback is open-loop for now: steppers report commanded step counts and servos report commanded angles
- AS5048A base/shoulder encoder readback is staged for known-pose and verification before any full closed-loop control.

## Quick Start After Restart

`localhost` only works while the FastAPI server is running. After restarting the PC, the old server process is gone, so start it again from PowerShell:

```powershell
cd "C:\Users\chark\Desktop\DTU\4 Semester\Mechatronics\Mechatronics Project Files\pc_app"
.\.venv\Scripts\Activate.ps1
uvicorn app.main:app --reload --host 127.0.0.1 --port 8000
```

Then open:

```text
http://127.0.0.1:8000
```

If the virtual environment does not exist yet, run the full setup below first.

## First-Time Setup

From this folder:

```powershell
cd pc_app
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
```

Run the app whenever you want to use the dashboard:

```powershell
uvicorn app.main:app --reload --host 127.0.0.1 --port 8000
```

Open:

```text
http://127.0.0.1:8000
```

Simulation mode is enabled by default in `config/robot.example.yaml`, so the dashboard should work without an ESP32-S3 connected. Machine-specific values are saved to `config/robot.local.yaml` when present.

## Startup Troubleshooting

If `http://127.0.0.1:8000` does not load:

1. Make sure the PowerShell window running `uvicorn` is still open.
2. Check that you are in the `pc_app` folder before running the command.
3. Activate the venv with `.\.venv\Scripts\Activate.ps1`.
4. If PowerShell blocks activation, run:

```powershell
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
```

5. If port 8000 is already in use, use another port:

```powershell
uvicorn app.main:app --reload --host 127.0.0.1 --port 8001
```

Then open `http://127.0.0.1:8001`.

6. If dependencies are missing, reinstall them:

```powershell
python -m pip install -r requirements.txt
```

## User Guide

The dashboard is a sandbox for testing the arm model and motion behavior before trusting real hardware.

### Layout

- Left panel: operation tabs and settings.
- Right viewport: live 3D robot view, preview ghost arm, target marker, and path line.
- Top-right rail: simulation, serial picker, disconnect, home, stop, diagnostics, and armed toggle.
- View widgets: HUD, target faders, preview/path/frame toggles, reset view, and Live Real.

### Control Tab

Use this for direct joint-angle experiments.

- Move joint sliders or type angles to update the preview ghost.
- `Apply` sends the current joint preview through the backend motion path.
- The preview stays visible until the reported pose reaches the commanded target, a newer target replaces it, or you press `Reset`.
- `Live joint jog` sends rate-limited joint updates while editing.
- The Tool panel switches between gripper controls and magnet controls from saved tool config.
- Keep movements small when the physical arm is connected.

### Tasks Tab

Use this for demo workflows.

- Select `Color Sorting` or `Pick and Place`.
- Refresh the camera panel to run current color/blob detection.
- Preview builds the pick/place sequence before execution.
- Detected objects can be shown in the 3D view when camera calibration is available.

### Kinematics Tab

Use this for Cartesian target experiments.

- Set `x`, `y`, `z` in millimeters and `phi` in degrees.
- Sliders and viewport faders auto-preview the target while you move them.
- `Mode` selects joint-space or Cartesian linear path generation.
- `Branch` selects the numerical IK seed preference.
- `Preview` builds a path and updates the ghost arm, target marker, and path line.
- `Execute` runs the accepted preview.
- The Standard DH table is editable and saved through Settings.

### Program Tab

Use this for temporary waypoint experiments. Programs are in-memory only and are not saved.

- Add the current joint pose or current IK target as a waypoint.
- Reorder or delete waypoints.
- `Preview Program` builds the full path.
- `Execute Program` runs the accepted program preview.

### Settings Tab

Use this as the shared robot model for both FK and IK.

- Edit link lengths, joint limits, home pose, zero offsets, direction signs, and motion defaults.
- Edit Hardware IO placeholders for stepper pins, TB6600 microstep value, gear ratios, servo pulse range, and enabled axes.
- Draft edits do not affect FK/IK until you press `Save`.
- After saving, the backend reloads config, refreshes the dashboard from the saved model, and tries to resync the ESP if serial hardware is connected.

### Hardware IO

The Settings tab is the single source of truth for physical pin and actuator mapping.

- All axes start disabled with unknown pins set to `-1`.
- Base and shoulder are currently modeled as steppers.
- Base and shoulder stepper drivers are currently modeled as TB6600 drivers; microstepping is set on physical DIP switches but still stored as a numeric value for steps-per-degree math.
- Elbow and wrist are currently modeled as 270 degree servos.
- Disabled axes are simulated by the controller and shown as `simulated`.
- Enabled valid axes are shown as `hardware`.
- A mix of physical and simulated axes is shown as `mixed`.
- Enabled axes with missing or invalid pins are shown as `invalid` and block hardware arming.
- `Sync Hardware` sends the saved config to the ESP and checks for acknowledgement.

### Live Real And Safety

Working assumptions:

- Simulation can preview and execute without hardware.
- Real hardware movement requires the backend to be connected to the controller.
- Real hardware movement requires hardware config sync to be `synced`.
- `Armed` is required before hardware execution or Live Real hardware movement.
- `Live Real` is always off after page reload and disconnect.
- `Stop` cancels active movement.

Use `Preview` first, check the ghost/path visually, then arm and execute only when the physical workspace is clear.

## Tests

```powershell
cd pc_app
pytest
```

The first tests cover config loading, joint limit validation, emergency-stop behavior, smoothing/rate limiting, FK, and protocol parsing/formatting.

## Configuration

Use `config/robot.example.yaml` as the tracked template. The app prefers `config/robot.local.yaml` when it exists and saves measured calibration there so private hardware values do not have to be committed.

Important fields:

- `links_mm`: link dimensions in millimeters
- `joints[].limits_deg`: conservative joint limits in degrees
- `joints[].home_deg`: configured home/reset pose
- `joints[].max_speed_deg_s`: per-joint speed limit
- `motion.smoothing_alpha`: smoothing applied to target changes
- `motion.acceleration_deg_s2`: simple acceleration limit
- `motion.allow_sudden_jumps`: keep `false` unless intentionally testing jumps
- `serial.port` and `serial.baud_rate`: initial USB serial settings
- `kinematics.dh_rows`: Standard DH rows used for FK and Jacobian IK
- `joints[].hardware.stepper`: step/dir/enable pins, driver model, full steps per rev, microsteps, and motor-to-joint gear ratio
- `joints[].hardware.servo`: PWM pin, pulse min/max, PWM frequency, servo range, neutral angle, and servo-to-joint gear ratio
- `tools`: active gripper or magnet dimensions and IO settings
- `encoders`: staged AS5048A readback and verification settings

## Coordinate Frame And Kinematics

The kinematics model uses Standard DH rows from config. The legacy link dimensions are kept for compatibility and for generating default DH rows.

Working assumption:

- Origin is the center of the base rotation axis on the mounting plane
- +Z points upward
- The arm points along global +Y when the base joint is 0 deg
- +X is horizontal sideways after base rotation
- Shoulder, elbow, and wrist are pitch joints in the vertical radial plane
- Shoulder 0 deg points the upper arm vertically upward
- Positive shoulder, elbow, and wrist angles bend the chain toward the local horizontal reach direction
- `phi` is the current tool angle target used by the numerical IK workflow

Lengths are in millimeters internally. Trigonometry uses radians internally. UI and config use degrees for joint angles.

## Calibration Notes

Before using real hardware, measure and update:

- Link lengths as center-to-center joint distances
- Base height from mounting plane to shoulder axis
- Tool/end-effector offset from wrist axis
- Joint zero angles
- Positive rotation direction for each joint
- Conservative software joint limits
- Stepper driver type, microstepping, gear ratio, and steps per revolution
- Stepper degrees per step at the joint after gearing
- Servo pulse min/max values for safe mechanical range
- Servo angle mapping and any gear ratio between servo and joint
- Homing switch or hard-limit availability

Recommended calibration workflow:

1. Set all joint limits narrower than the physical range.
2. Define a repeatable zero pose for each joint.
3. Verify positive direction with small manual moves.
4. Measure several physical end-effector positions.
5. Compare measured positions against FK.
6. Adjust link lengths and zero offsets.
7. Only widen limits after repeated safe tests.

## Serial And Hardware Mode

There are currently three ESP-side firmware choices in `../controller_firmware/platformio`:

- `main.cpp`: preserved single-axis stepper/servo test firmware.
- `protocol_stub.cpp`: no-motor safe protocol parser.
- `arm_controller.cpp`: full-arm open-loop controller that accepts dashboard config and moves only enabled valid axes.

The current line-based protocol is documented in `../controller_firmware/protocol_stub.md`.

Core commands:

```text
HELLO
STATUS
CONFIG BEGIN / CONFIG JOINT / CONFIG END
ARM 0|1
SETPOSE j1 j2 j3 j4
MOVEJ j1 j2 j3 j4 speed accel
STOP
ESTOP
HOME
TOOL OPEN|CLOSE
TOOL SET value=0.000
TOOL ON|OFF
```

Current status response:

```text
STATUS state=idle homed=0 known=0 pose_source=unknown armed=0 hw=mixed enabled=1000 enc=0000 e1=0.0 e2=0.0 j1=0.0 j2=20.0 j3=20.0 j4=0.0 closed_loop=off tool_type=generic tool=unknown tool_value=0.000 fault=OK
```

Optional/newer status fields include `known=0|1`, `pose_source=<manual|setpose|encoder|mixed>`, `enc=1100`, `e1=<deg>`,
`e2=<deg>`, `closed_loop=<off|readback|settle_correction>`, `tool_type=<generic|servo_gripper|electromagnet>`,
`tool=<open|closed|on|off|moving|unknown>`, and `tool_value=<0.000..1.000>`.

Working assumption: the PC remains the planner. The ESP is a safe target follower for PC-streamed `MOVEJ` waypoints.

## Bluetooth Notes

Bluetooth is intentionally not implemented yet.

Later, add a Bluetooth transport that implements the same high-level interface as the serial transport. The robot state, safety checks, FK, motion smoothing, and UI should not need to change when the transport changes.
