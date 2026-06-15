from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


CONFIG_DIR = Path(__file__).resolve().parents[1] / "config"
EXAMPLE_CONFIG_PATH = CONFIG_DIR / "robot.example.yaml"
LOCAL_CONFIG_PATH = CONFIG_DIR / "robot.local.yaml"
CONFIG_PATH = EXAMPLE_CONFIG_PATH


@dataclass(frozen=True)
class StepperHardwareConfig:
    enabled: bool
    step_pin: int
    dir_pin: int
    enable_pin: int
    enable_active_low: bool
    m0_pin: int
    m1_pin: int
    m2_pin: int
    driver_model: str
    motor_full_steps_per_rev: int
    microsteps: int
    gear_ratio: float


@dataclass(frozen=True)
class ServoHardwareConfig:
    enabled: bool
    pwm_pin: int
    pulse_min_us: int
    pulse_max_us: int
    pwm_frequency_hz: int
    servo_range_deg: float
    neutral_deg: float
    gear_ratio: float


@dataclass(frozen=True)
class JointHardwareConfig:
    stepper: StepperHardwareConfig | None = None
    servo: ServoHardwareConfig | None = None


@dataclass(frozen=True)
class JointConfig:
    name: str
    actuator: str
    min_deg: float
    max_deg: float
    home_deg: float
    max_speed_deg_s: float
    max_accel_deg_s2: float
    zero_offset_deg: float
    direction_sign: int
    hardware: JointHardwareConfig


@dataclass(frozen=True)
class LinkConfig:
    base_height_mm: float
    upper_arm_mm: float
    forearm_mm: float
    wrist_mm: float
    tool_mm: float


@dataclass(frozen=True)
class MotionConfig:
    update_rate_hz: float
    smoothing_alpha: float
    command_rate_limit_hz: float
    acceleration_deg_s2: float
    allow_sudden_jumps: bool


@dataclass(frozen=True)
class SerialConfig:
    port: str
    baud_rate: int
    timeout_s: float


@dataclass(frozen=True)
class RobotConfig:
    joints: list[JointConfig]
    links: LinkConfig
    motion: MotionConfig
    serial: SerialConfig
    simulation_default: bool
    coordinate_frame_notes: str
    raw: dict[str, Any]
    source_path: Path

    @property
    def joint_names(self) -> list[str]:
        return [joint.name for joint in self.joints]

    @property
    def home_pose(self) -> list[float]:
        return [joint.home_deg for joint in self.joints]


def _require_number(value: Any, name: str) -> float:
    if not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be numeric")
    return float(value)


def _direction_sign(value: Any, name: str) -> int:
    if value in {-1, "-1", "reverse", "REV", "rev"}:
        return -1
    if value in {1, "1", "forward", "FWD", "fwd"}:
        return 1
    raise ValueError(f"{name} must be 1 or -1")


def _int_value(value: Any, name: str) -> int:
    if value is None or value in {"", "TBD", "tbd", "unknown", "UNKNOWN"}:
        return -1
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be an integer")
    return int(value)


def _positive_int(value: Any, name: str) -> int:
    result = _int_value(value, name)
    if result <= 0:
        raise ValueError(f"{name} must be positive")
    return result


def _hardware_for_joint(item: dict[str, Any], actuator: str, name: str) -> JointHardwareConfig:
    hardware_raw = item.get("hardware")
    if not isinstance(hardware_raw, dict):
        placeholder = item.get("calibration_placeholder", {})
        if actuator == "servo":
            hardware_raw = {
                "servo": {
                    "enabled": False,
                    "pwm_pin": placeholder.get("servo_pin", -1),
                }
            }
        else:
            hardware_raw = {
                "stepper": {
                    "enabled": False,
                    "step_pin": placeholder.get("step_pin", -1),
                    "dir_pin": placeholder.get("dir_pin", -1),
                    "enable_pin": placeholder.get("enable_pin", -1),
                }
            }

    stepper: StepperHardwareConfig | None = None
    servo: ServoHardwareConfig | None = None
    if actuator == "stepper":
        raw = hardware_raw.get("stepper", {}) if isinstance(hardware_raw.get("stepper", {}), dict) else {}
        stepper = StepperHardwareConfig(
            enabled=bool(raw.get("enabled", False)),
            step_pin=_int_value(raw.get("step_pin", -1), f"{name}.hardware.stepper.step_pin"),
            dir_pin=_int_value(raw.get("dir_pin", -1), f"{name}.hardware.stepper.dir_pin"),
            enable_pin=_int_value(raw.get("enable_pin", -1), f"{name}.hardware.stepper.enable_pin"),
            enable_active_low=bool(raw.get("enable_active_low", True)),
            m0_pin=_int_value(raw.get("m0_pin", -1), f"{name}.hardware.stepper.m0_pin"),
            m1_pin=_int_value(raw.get("m1_pin", -1), f"{name}.hardware.stepper.m1_pin"),
            m2_pin=_int_value(raw.get("m2_pin", -1), f"{name}.hardware.stepper.m2_pin"),
            driver_model=str(raw.get("driver_model", "TBD")),
            motor_full_steps_per_rev=_positive_int(
                raw.get("motor_full_steps_per_rev", 200),
                f"{name}.hardware.stepper.motor_full_steps_per_rev",
            ),
            microsteps=_positive_int(raw.get("microsteps", 16), f"{name}.hardware.stepper.microsteps"),
            gear_ratio=_require_number(raw.get("gear_ratio", 1.0), f"{name}.hardware.stepper.gear_ratio"),
        )
        if stepper.gear_ratio <= 0:
            raise ValueError(f"{name}.hardware.stepper.gear_ratio must be positive")
    elif actuator == "servo":
        raw = hardware_raw.get("servo", {}) if isinstance(hardware_raw.get("servo", {}), dict) else {}
        servo = ServoHardwareConfig(
            enabled=bool(raw.get("enabled", False)),
            pwm_pin=_int_value(raw.get("pwm_pin", -1), f"{name}.hardware.servo.pwm_pin"),
            pulse_min_us=_positive_int(raw.get("pulse_min_us", 500), f"{name}.hardware.servo.pulse_min_us"),
            pulse_max_us=_positive_int(raw.get("pulse_max_us", 2500), f"{name}.hardware.servo.pulse_max_us"),
            pwm_frequency_hz=_positive_int(
                raw.get("pwm_frequency_hz", 50), f"{name}.hardware.servo.pwm_frequency_hz"
            ),
            servo_range_deg=_require_number(raw.get("servo_range_deg", 270.0), f"{name}.hardware.servo.servo_range_deg"),
            neutral_deg=_require_number(raw.get("neutral_deg", 135.0), f"{name}.hardware.servo.neutral_deg"),
            gear_ratio=_require_number(raw.get("gear_ratio", 1.0), f"{name}.hardware.servo.gear_ratio"),
        )
        if servo.pulse_min_us >= servo.pulse_max_us:
            raise ValueError(f"{name}.hardware.servo.pulse_min_us must be below pulse_max_us")
        if servo.servo_range_deg <= 0:
            raise ValueError(f"{name}.hardware.servo.servo_range_deg must be positive")
        if servo.gear_ratio <= 0:
            raise ValueError(f"{name}.hardware.servo.gear_ratio must be positive")
    return JointHardwareConfig(stepper=stepper, servo=servo)


def active_config_path() -> Path:
    return LOCAL_CONFIG_PATH if LOCAL_CONFIG_PATH.exists() else EXAMPLE_CONFIG_PATH


def ensure_local_config() -> Path:
    if not LOCAL_CONFIG_PATH.exists():
        LOCAL_CONFIG_PATH.write_text(EXAMPLE_CONFIG_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    return LOCAL_CONFIG_PATH


def load_config(path: str | Path | None = None) -> RobotConfig:
    config_path = Path(path) if path is not None else active_config_path()
    with config_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}

    motion_raw = raw.get("motion", {})
    default_joint_accel = _require_number(
        motion_raw.get("acceleration_deg_s2", 120), "motion.acceleration_deg_s2"
    )

    joint_items = raw.get("joints", [])
    if len(joint_items) != 4:
        raise ValueError("config must define exactly four joints")

    joints: list[JointConfig] = []
    for index, item in enumerate(joint_items):
        limits = item.get("limits_deg", {})
        name = item.get("name", f"joint_{index + 1}")
        min_deg = _require_number(limits.get("min"), f"{name}.limits_deg.min")
        max_deg = _require_number(limits.get("max"), f"{name}.limits_deg.max")
        home_deg = _require_number(item.get("home_deg", 0.0), f"{name}.home_deg")
        max_speed = _require_number(
            item.get("max_speed_deg_s", 20.0), f"{name}.max_speed_deg_s"
        )
        max_accel = _require_number(
            item.get("max_accel_deg_s2", default_joint_accel), f"{name}.max_accel_deg_s2"
        )
        zero_offset = _require_number(
            item.get("zero_offset_deg", 0.0), f"{name}.zero_offset_deg"
        )
        direction = _direction_sign(item.get("direction_sign", 1), f"{name}.direction_sign")
        if min_deg >= max_deg:
            raise ValueError(f"{name} min limit must be below max limit")
        if not min_deg <= home_deg <= max_deg:
            raise ValueError(f"{name} home pose is outside joint limits")
        if max_speed <= 0:
            raise ValueError(f"{name} max_speed_deg_s must be positive")
        if max_accel <= 0:
            raise ValueError(f"{name} max_accel_deg_s2 must be positive")
        actuator = str(item.get("actuator", "unknown"))
        joints.append(
            JointConfig(
                name=str(name),
                actuator=actuator,
                min_deg=min_deg,
                max_deg=max_deg,
                home_deg=home_deg,
                max_speed_deg_s=max_speed,
                max_accel_deg_s2=max_accel,
                zero_offset_deg=zero_offset,
                direction_sign=direction,
                hardware=_hardware_for_joint(item, actuator, str(name)),
            )
        )

    links_raw = raw.get("links_mm", {})
    links = LinkConfig(
        base_height_mm=_require_number(links_raw.get("base_height", 80.0), "base_height"),
        upper_arm_mm=_require_number(links_raw.get("upper_arm", 140.0), "upper_arm"),
        forearm_mm=_require_number(links_raw.get("forearm", 120.0), "forearm"),
        wrist_mm=_require_number(links_raw.get("wrist", 60.0), "wrist"),
        tool_mm=_require_number(links_raw.get("tool", 30.0), "tool"),
    )

    motion = MotionConfig(
        update_rate_hz=_require_number(motion_raw.get("update_rate_hz", 30), "update_rate_hz"),
        smoothing_alpha=_require_number(
            motion_raw.get("smoothing_alpha", 0.35), "smoothing_alpha"
        ),
        command_rate_limit_hz=_require_number(
            motion_raw.get("command_rate_limit_hz", 12), "command_rate_limit_hz"
        ),
        acceleration_deg_s2=_require_number(
            motion_raw.get("acceleration_deg_s2", 120), "acceleration_deg_s2"
        ),
        allow_sudden_jumps=bool(motion_raw.get("allow_sudden_jumps", False)),
    )
    if motion.update_rate_hz <= 0:
        raise ValueError("motion.update_rate_hz must be positive")
    if not 0 < motion.smoothing_alpha <= 1:
        raise ValueError("motion.smoothing_alpha must be in (0, 1]")
    if motion.command_rate_limit_hz <= 0:
        raise ValueError("motion.command_rate_limit_hz must be positive")

    serial_raw = raw.get("serial", {})
    serial = SerialConfig(
        port=str(serial_raw.get("port", "COM6")),
        baud_rate=int(serial_raw.get("baud_rate", 115200)),
        timeout_s=_require_number(serial_raw.get("timeout_s", 0.2), "serial.timeout_s"),
    )

    return RobotConfig(
        joints=joints,
        links=links,
        motion=motion,
        serial=serial,
        simulation_default=bool(raw.get("simulation_default", True)),
        coordinate_frame_notes=str(raw.get("coordinate_frame_notes", "")),
        raw=raw,
        source_path=config_path,
    )


def save_calibration_updates(path: str | Path, updates: dict[str, Any]) -> None:
    """Persist editable calibration values while keeping comments in the YAML file."""
    try:
        from ruamel.yaml import YAML
    except ImportError as exc:  # pragma: no cover - exercised only in misconfigured envs.
        raise RuntimeError("ruamel.yaml is required to preserve config comments") from exc

    config_path = Path(path)
    yaml_rt = YAML()
    yaml_rt.preserve_quotes = True
    yaml_rt.indent(mapping=2, sequence=4, offset=2)

    with config_path.open("r", encoding="utf-8") as handle:
        data = yaml_rt.load(handle) or {}

    links = updates.get("links_mm")
    if isinstance(links, dict):
        data.setdefault("links_mm", {})
        for key in ["base_height", "upper_arm", "forearm", "wrist", "tool"]:
            if key in links:
                data["links_mm"][key] = float(links[key])

    joints = updates.get("joints")
    if isinstance(joints, list):
        data.setdefault("joints", [])
        for index, patch in enumerate(joints):
            if index >= len(data["joints"]) or not isinstance(patch, dict):
                continue
            joint = data["joints"][index]
            if "limits_deg" in patch and isinstance(patch["limits_deg"], dict):
                joint.setdefault("limits_deg", {})
                limits = patch["limits_deg"]
                if "min" in limits:
                    joint["limits_deg"]["min"] = float(limits["min"])
                if "max" in limits:
                    joint["limits_deg"]["max"] = float(limits["max"])
            for key in [
                "home_deg",
                "max_speed_deg_s",
                "max_accel_deg_s2",
                "zero_offset_deg",
            ]:
                if key in patch:
                    joint[key] = float(patch[key])
            if "direction_sign" in patch:
                joint["direction_sign"] = _direction_sign(
                    patch["direction_sign"], f"joints[{index}].direction_sign"
                )
            if "hardware" in patch and isinstance(patch["hardware"], dict):
                joint.setdefault("hardware", {})
                hardware = patch["hardware"]
                if "stepper" in hardware and isinstance(hardware["stepper"], dict):
                    stepper = joint["hardware"].setdefault("stepper", {})
                    for key in [
                        "enabled",
                        "step_pin",
                        "dir_pin",
                        "enable_pin",
                        "enable_active_low",
                        "m0_pin",
                        "m1_pin",
                        "m2_pin",
                        "driver_model",
                        "motor_full_steps_per_rev",
                        "microsteps",
                        "gear_ratio",
                    ]:
                        if key in hardware["stepper"]:
                            stepper[key] = hardware["stepper"][key]
                if "servo" in hardware and isinstance(hardware["servo"], dict):
                    servo = joint["hardware"].setdefault("servo", {})
                    for key in [
                        "enabled",
                        "pwm_pin",
                        "pulse_min_us",
                        "pulse_max_us",
                        "pwm_frequency_hz",
                        "servo_range_deg",
                        "neutral_deg",
                        "gear_ratio",
                    ]:
                        if key in hardware["servo"]:
                            servo[key] = hardware["servo"][key]

    motion = updates.get("motion")
    if isinstance(motion, dict):
        data.setdefault("motion", {})
        for key in [
            "update_rate_hz",
            "smoothing_alpha",
            "command_rate_limit_hz",
            "acceleration_deg_s2",
        ]:
            if key in motion:
                data["motion"][key] = float(motion[key])
        if "allow_sudden_jumps" in motion:
            data["motion"]["allow_sudden_jumps"] = bool(motion["allow_sudden_jumps"])

    for key in ["named_positions", "camera", "color_profiles", "drop_zones", "task_defaults", "tool"]:
        if key in updates and isinstance(updates[key], dict):
            data[key] = updates[key]

    with config_path.open("w", encoding="utf-8") as handle:
        yaml_rt.dump(data, handle)
