from fastapi.testclient import TestClient

import app.main as main
from app.robot_state import MotionState


def reset_runtime_state() -> None:
    main.cancel_motion_tasks()
    main.state.connected = True
    main.state.simulation = True
    main.state.hardware_armed = False
    main.state.live_motion_enabled = False
    main.state.motion_state = MotionState.IDLE
    main.state.reported_angles_deg = main.config.home_pose.copy()
    main.state.target_angles_deg = main.config.home_pose.copy()
    main.state.clear_error()


def test_live_motion_hardware_requires_armed_toggle():
    reset_runtime_state()
    main.state.simulation = False
    main.state.connected = True
    main.state.hardware_armed = False
    client = TestClient(main.app)

    response = client.post("/api/live-motion", json={"enabled": True})

    assert response.status_code == 200
    payload = response.json()
    assert not payload["ok"]
    assert "Armed" in payload["error"]
    assert not payload["state"]["live_motion_enabled"]


def test_live_target_requires_live_motion_enabled():
    reset_runtime_state()
    client = TestClient(main.app)

    response = client.post("/api/live-target", json={"angles_deg": main.config.home_pose})

    assert response.status_code == 200
    payload = response.json()
    assert not payload["ok"]
    assert "disabled" in payload["error"]


def test_live_target_accepts_simulation_joint_target_when_enabled():
    reset_runtime_state()
    client = TestClient(main.app)
    enabled = client.post("/api/live-motion", json={"enabled": True}).json()
    assert enabled["ok"]
    target = main.config.home_pose.copy()
    target[0] += 2.0

    response = client.post(
        "/api/live-target",
        json={
            "angles_deg": target,
            "settings": {"global_speed_deg_s": 20.0, "global_accel_deg_s2": 100.0},
        },
    )

    payload = response.json()
    assert payload["ok"]
    assert payload["preview"]["mode"] == "jog"
    assert payload["preview"]["trajectory"]["waypoints"][-1] == target
    reset_runtime_state()
