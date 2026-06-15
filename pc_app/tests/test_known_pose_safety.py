from fastapi.testclient import TestClient

import app.main as main
from app.robot_state import MotionState


def test_hardware_motion_requires_known_pose():
    main.cancel_motion_tasks()
    main.state.connected = True
    main.state.simulation = False
    main.state.hardware_armed = True
    main.state.known_pose = False
    main.state.config_sync_status = "synced"
    main.state.motion_state = MotionState.IDLE
    main.state.clear_error()
    client = TestClient(main.app)

    response = client.post("/api/joints", json={"angles_deg": main.config.home_pose})

    payload = response.json()
    assert not payload["ok"]
    assert "pose is unknown" in payload["error"]
    main.state.simulation = True
    main.state.known_pose = True
