from fastapi.testclient import TestClient

import app.main as main


def test_program_preview_returns_revision_and_step_summary():
    main.cancel_motion_tasks()
    main.path_previews.clear()
    main.state.connected = True
    main.state.simulation = True
    main.state.clear_error()
    client = TestClient(main.app)

    response = client.post(
        "/api/path/preview",
        json={
            "mode": "program",
            "program_revision": 7,
            "waypoints": [
                {
                    "label": "disabled draft",
                    "type": "joint",
                    "mode": "joint",
                    "enabled": False,
                    "angles_deg": main.state.reported_angles_deg,
                },
                {
                    "label": "current pose",
                    "type": "joint",
                    "mode": "joint",
                    "enabled": True,
                    "angles_deg": main.state.reported_angles_deg,
                },
            ],
        },
    )

    payload = response.json()
    assert payload["ok"], payload
    preview = payload["preview"]
    assert preview["program_revision"] == 7
    assert preview["trajectory"]["step_count"] == 2
    assert preview["trajectory"]["move_count"] == 1
    assert [item["status"] for item in preview["trajectory"]["step_results"]] == ["disabled", "valid"]


def test_program_execute_rejects_a_revision_that_does_not_match_preview():
    main.cancel_motion_tasks()
    preview_id = "program-revision-test"
    main.path_previews[preview_id] = {
        "id": preview_id,
        "mode": "program",
        "program_revision": 3,
        "trajectory": {"mode": "program"},
    }
    client = TestClient(main.app)

    try:
        payload = client.post(
            "/api/path/execute",
            json={"preview_id": preview_id, "program_revision": 4},
        ).json()
    finally:
        main.path_previews.pop(preview_id, None)

    assert not payload["ok"]
    assert "changed since preview" in payload["error"]
