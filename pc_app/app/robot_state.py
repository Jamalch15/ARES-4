from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum
from time import time
from typing import Any


class MotionState(StrEnum):
    IDLE = "idle"
    MOVING = "moving"
    STOPPED = "stopped"
    ESTOP = "estop"
    FAULT = "fault"


@dataclass
class RobotState:
    joint_names: list[str]
    target_angles_deg: list[float]
    reported_angles_deg: list[float]
    connected: bool = False
    simulation: bool = True
    serial_port: str | None = None
    motion_state: MotionState = MotionState.IDLE
    last_command: str = ""
    last_error: str = ""
    homed: bool = False
    hardware_armed: bool = False
    live_motion_enabled: bool = False
    hardware_mode: str = "simulated"
    hardware_enabled_axes: str = "0000"
    hardware_axis_states: list[str] = field(default_factory=lambda: ["simulated"] * 4)
    config_sync_status: str = "not_connected"
    config_sync_message: str = ""
    known_pose: bool = True
    encoder_available: str = "0000"
    encoder_angles_deg: list[float | None] = field(default_factory=lambda: [None] * 4)
    tool_state: str = "unknown"
    tool_value: float | None = None
    fk: dict[str, float] = field(default_factory=dict)
    updated_at: float = field(default_factory=time)

    def to_dict(self) -> dict[str, Any]:
        return {
            "joint_names": self.joint_names,
            "target_angles_deg": self.target_angles_deg,
            "reported_angles_deg": self.reported_angles_deg,
            "connected": self.connected,
            "simulation": self.simulation,
            "serial_port": self.serial_port,
            "motion_state": self.motion_state.value,
            "last_command": self.last_command,
            "last_error": self.last_error,
            "homed": self.homed,
            "hardware_armed": self.hardware_armed,
            "live_motion_enabled": self.live_motion_enabled,
            "hardware_mode": self.hardware_mode,
            "hardware_enabled_axes": self.hardware_enabled_axes,
            "hardware_axis_states": self.hardware_axis_states,
            "config_sync_status": self.config_sync_status,
            "config_sync_message": self.config_sync_message,
            "known_pose": self.known_pose,
            "encoder_available": self.encoder_available,
            "encoder_angles_deg": self.encoder_angles_deg,
            "tool_state": self.tool_state,
            "tool_value": self.tool_value,
            "fk": self.fk,
            "updated_at": self.updated_at,
        }

    def set_error(self, message: str, fault: bool = False) -> None:
        self.last_error = message
        if fault:
            self.motion_state = MotionState.FAULT
        self.updated_at = time()

    def clear_error(self) -> None:
        self.last_error = ""
        self.updated_at = time()
