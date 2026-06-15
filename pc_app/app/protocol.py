from __future__ import annotations

from dataclasses import dataclass

from .config import JointConfig


def format_hello() -> str:
    return "HELLO"


def format_status() -> str:
    return "STATUS"


def format_movej(joints_deg: list[float], speed: float, accel: float) -> str:
    if len(joints_deg) != 4:
        raise ValueError("MOVEJ requires exactly four joint angles")
    values = " ".join(f"{angle:.3f}" for angle in joints_deg)
    return f"MOVEJ {values} {speed:.3f} {accel:.3f}"


def format_stop() -> str:
    return "STOP"


def format_estop() -> str:
    return "ESTOP"


def format_home() -> str:
    return "HOME"


def format_arm(armed: bool) -> str:
    return f"ARM {1 if armed else 0}"


def format_setpose(joints_deg: list[float]) -> str:
    if len(joints_deg) != 4:
        raise ValueError("SETPOSE requires exactly four joint angles")
    values = " ".join(f"{angle:.3f}" for angle in joints_deg)
    return f"SETPOSE {values}"


def format_tool(action: str, value: float | None = None) -> str:
    normalized = action.strip().upper()
    if normalized in {"OPEN", "CLOSE"}:
        return f"TOOL {normalized}"
    if normalized == "SET":
        if value is None:
            raise ValueError("TOOL SET requires a value")
        clamped = max(0.0, min(1.0, float(value)))
        return f"TOOL SET value={clamped:.3f}"
    raise ValueError(f"unsupported TOOL action {action}")


def format_config_lines(joints: list[JointConfig]) -> list[str]:
    if len(joints) != 4:
        raise ValueError("hardware CONFIG requires exactly four joints")
    lines = ["CONFIG BEGIN axes=4"]
    for index, joint in enumerate(joints, start=1):
        common = (
            f"index={index} name={joint.name} actuator={joint.actuator} "
            f"zero={joint.zero_offset_deg:.3f} sign={joint.direction_sign} "
            f"min={joint.min_deg:.3f} max={joint.max_deg:.3f} home={joint.home_deg:.3f} "
            f"max_speed={joint.max_speed_deg_s:.3f} max_accel={joint.max_accel_deg_s2:.3f}"
        )
        if joint.actuator == "servo" and joint.hardware.servo:
            servo = joint.hardware.servo
            lines.append(
                "CONFIG JOINT "
                f"{common} enabled={1 if servo.enabled else 0} "
                f"pwm={servo.pwm_pin} min_us={servo.pulse_min_us} max_us={servo.pulse_max_us} "
                f"freq={servo.pwm_frequency_hz} servo_range={servo.servo_range_deg:.3f} "
                f"neutral={servo.neutral_deg:.3f} gear={servo.gear_ratio:.6f}"
            )
        else:
            stepper = joint.hardware.stepper
            if stepper is None:
                raise ValueError(f"{joint.name} has no stepper hardware config")
            driver_model = stepper.driver_model.replace(" ", "_")
            lines.append(
                "CONFIG JOINT "
                f"{common} enabled={1 if stepper.enabled else 0} "
                f"step={stepper.step_pin} dir={stepper.dir_pin} enable={stepper.enable_pin} "
                f"enable_low={1 if stepper.enable_active_low else 0} "
                f"m0={stepper.m0_pin} m1={stepper.m1_pin} m2={stepper.m2_pin} "
                f"driver={driver_model} full_steps={stepper.motor_full_steps_per_rev} "
                f"microsteps={stepper.microsteps} gear={stepper.gear_ratio:.6f}"
            )
    lines.append("CONFIG END")
    return lines


@dataclass(frozen=True)
class ControllerStatus:
    state: str
    homed: bool
    joints_deg: list[float]
    fault: str
    armed: bool = False
    hardware_mode: str = "unknown"
    enabled_axes: str = "0000"
    known_pose: bool = False
    encoder_available: str = "0000"
    encoder_angles_deg: list[float | None] | None = None
    tool_state: str = "unknown"


def parse_status(line: str) -> ControllerStatus:
    parts = line.strip().split()
    if not parts or parts[0] != "STATUS":
        raise ValueError("status line must start with STATUS")

    values: dict[str, str] = {}
    for token in parts[1:]:
        if "=" in token:
            key, value = token.split("=", 1)
            values[key] = value

    joints = [float(values.get(f"j{index}", "0.0")) for index in range(1, 5)]
    encoder_angles: list[float | None] = []
    for index in range(1, 5):
        key = f"e{index}"
        encoder_angles.append(float(values[key]) if key in values else None)
    homed = values.get("homed", "0") in {"1", "true", "True"}
    return ControllerStatus(
        state=values.get("state", "unknown"),
        homed=homed,
        joints_deg=joints,
        fault=values.get("fault", "UNKNOWN"),
        armed=values.get("armed", "0") in {"1", "true", "True"},
        hardware_mode=values.get("hw", values.get("hardware", "unknown")),
        enabled_axes=values.get("enabled", "0000"),
        known_pose=values.get("known", "1" if homed else "0") in {"1", "true", "True"},
        encoder_available=values.get("enc", "0000"),
        encoder_angles_deg=encoder_angles,
        tool_state=values.get("tool", "unknown"),
    )
