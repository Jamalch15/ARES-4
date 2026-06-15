from dataclasses import replace

from fastapi.testclient import TestClient

import app.main as main
from app.robot_state import MotionState


class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
        self.sent = []
        self.connection = object()

    @property
    def is_connected(self):
        return True

    def send_line(self, line):
        self.sent.append(line)

    def read_line(self):
        return self.responses.pop(0) if self.responses else ""

    def clear_input(self):
        pass

    def read_until_prefix(self, prefix, timeout_s=2.0):
        while self.responses:
            line = self.read_line()
            if line.startswith(prefix):
                return line
        raise RuntimeError(f"timed out waiting for {prefix}")


def reset_runtime_state() -> None:
    main.cancel_motion_tasks()
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = False
    main.state.live_motion_enabled = False
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = main.config.home_pose.copy()
    main.state.target_angles_deg = main.config.home_pose.copy()
    main.state.config_sync_status = "stale"
    main.state.clear_error()


def disable_joint_hardware(joint):
    stepper = replace(joint.hardware.stepper, enabled=False) if joint.hardware.stepper else None
    servo = replace(joint.hardware.servo, enabled=False) if joint.hardware.servo else None
    return replace(joint, hardware=replace(joint.hardware, stepper=stepper, servo=servo))


def config_with_only_first_axis_enabled(config):
    joints = [disable_joint_hardware(joint) for joint in config.joints]
    first_joint = replace(
        joints[0],
        hardware=replace(
            joints[0].hardware,
            stepper=replace(
                joints[0].hardware.stepper,
                enabled=True,
                step_pin=17,
                dir_pin=16,
            ),
        ),
    )
    return replace(config, joints=[first_joint, *joints[1:]])


def test_hardware_sync_reports_synced(monkeypatch):
    reset_runtime_state()
    fake = FakeSerial(["OK command=CONFIG axes=4 hw=simulated enabled=0000"])
    monkeypatch.setattr(main, "serial_client", fake)
    client = TestClient(main.app)

    response = client.post("/api/hardware/sync")

    payload = response.json()
    assert payload["ok"]
    assert payload["status"] == "synced"
    assert fake.sent[0] == "CONFIG BEGIN axes=4"
    assert fake.sent[-1] == "CONFIG END"


def test_hardware_sync_reports_unsupported(monkeypatch):
    reset_runtime_state()
    fake = FakeSerial(["ERR code=UNKNOWN message=CONFIG"])
    monkeypatch.setattr(main, "serial_client", fake)
    client = TestClient(main.app)

    response = client.post("/api/hardware/sync")

    payload = response.json()
    assert not payload["ok"]
    assert payload["status"] == "unsupported"


def test_partial_hardware_sync_allows_arm_as_mixed(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_only_first_axis_enabled(original_config))
        reset_runtime_state()
        fake = FakeSerial(
            [
                "OK command=CONFIG axes=4 hw=mixed enabled=1000",
                "OK command=ARM armed=1",
                "STATUS state=idle homed=0 armed=1 hw=mixed enabled=1000 j1=0.0 j2=20.0 j3=20.0 j4=0.0 fault=OK",
            ]
        )
        monkeypatch.setattr(main, "serial_client", fake)
        client = TestClient(main.app)

        sync_response = client.post("/api/hardware/sync")
        arm_response = client.post("/api/hardware-arm", json={"armed": True})

        sync_payload = sync_response.json()
        arm_payload = arm_response.json()
        assert sync_payload["ok"]
        assert sync_payload["evaluation"]["mode"] == "mixed"
        assert arm_payload["ok"]
        assert arm_payload["state"]["hardware_armed"] is True
        assert arm_payload["state"]["hardware_mode"] == "mixed"
        assert "ARM 1" in fake.sent
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_unsupported_config_blocks_hardware_arm(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_only_first_axis_enabled(original_config))
        reset_runtime_state()
        main.state.config_sync_status = "unsupported"
        fake = FakeSerial([])
        monkeypatch.setattr(main, "serial_client", fake)
        client = TestClient(main.app)

        response = client.post("/api/hardware-arm", json={"armed": True})

        payload = response.json()
        assert not payload["ok"]
        assert "not synced" in payload["error"]
        assert fake.sent == []
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_hardware_motion_blocks_unsynced_config(monkeypatch):
    original_config = main.config
    try:
        monkeypatch.setattr(main, "config", config_with_only_first_axis_enabled(original_config))
        reset_runtime_state()
        main.state.hardware_armed = True
        client = TestClient(main.app)

        response = client.post("/api/joints", json={"angles_deg": main.config.home_pose})

        payload = response.json()
        assert not payload["ok"]
        assert "not synced" in payload["error"]
    finally:
        monkeypatch.setattr(main, "config", original_config)


def test_enabled_axis_with_missing_pins_is_invalid(monkeypatch):
    original_config = main.config
    try:
        first_joint = replace(
            original_config.joints[0],
            hardware=replace(
                original_config.joints[0].hardware,
                stepper=replace(original_config.joints[0].hardware.stepper, enabled=True, step_pin=-1, dir_pin=-1),
            ),
        )
        patched = replace(original_config, joints=[first_joint, *original_config.joints[1:]])
        monkeypatch.setattr(main, "config", patched)
        reset_runtime_state()
        client = TestClient(main.app)

        response = client.post("/api/hardware/sync")

        payload = response.json()
        assert not payload["ok"]
        assert payload["status"] == "invalid"
        assert payload["evaluation"]["mode"] == "invalid"
    finally:
        monkeypatch.setattr(main, "config", original_config)
