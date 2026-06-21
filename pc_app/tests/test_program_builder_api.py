import asyncio

from fastapi.testclient import TestClient

import app.main as main
import app.program_library as program_library


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


def test_program_step_preview_uses_preceding_steps_as_context():
    main.cancel_motion_tasks()
    main.path_previews.clear()
    main.state.connected = True
    main.state.simulation = True
    main.state.clear_error()
    client = TestClient(main.app)
    start = list(main.state.reported_angles_deg)
    intermediate = start.copy()
    intermediate[0] = min(main.config.joints[0].max_deg, start[0] + 5.0)

    payload = client.post(
        "/api/programs/preview-step",
        json={
            "step_index": 1,
            "program_revision": 12,
            "waypoints": [
                {
                    "label": "Intermediate",
                    "type": "joint",
                    "mode": "joint",
                    "enabled": True,
                    "angles_deg": intermediate,
                },
                {
                    "label": "Return",
                    "type": "joint",
                    "mode": "joint",
                    "enabled": True,
                    "angles_deg": start,
                },
                {
                    "label": "Not included",
                    "type": "joint",
                    "mode": "joint",
                    "enabled": True,
                    "angles_deg": intermediate,
                },
            ],
        },
    ).json()

    assert payload["ok"], payload
    assert payload["step_index"] == 1
    assert payload["preview"]["program_revision"] == 12
    assert payload["preview"]["trajectory"]["step_count"] == 2
    assert [result["label"] for result in payload["preview"]["trajectory"]["step_results"]] == [
        "Intermediate",
        "Return",
    ]


def test_program_preview_supports_end_effector_steps():
    main.cancel_motion_tasks()
    main.path_previews.clear()
    main.state.connected = True
    main.state.simulation = True
    main.state.clear_error()
    client = TestClient(main.app)

    payload = client.post(
        "/api/path/preview",
        json={
            "mode": "program",
            "program_revision": 21,
            "waypoints": [
                {
                    "label": "Current pose",
                    "type": "joint",
                    "angles_deg": main.state.reported_angles_deg,
                },
                {
                    "label": "Close gripper",
                    "type": "tool",
                    "tool": "gripper",
                    "action": "close",
                    "settle_ms": 150,
                },
            ],
        },
    ).json()

    assert payload["ok"], payload
    trajectory = payload["preview"]["trajectory"]
    assert trajectory["move_count"] == 1
    assert trajectory["action_count"] == 1
    assert [step["kind"] for step in trajectory["execution_steps"]] == ["motion", "tool"]
    assert trajectory["step_results"][1]["duration_s"] == 0.15


def test_program_sequence_executes_end_effector_action_in_simulation():
    main.cancel_motion_tasks()
    previous_tool_state = main.state.tool_state
    previous_tool_value = main.state.tool_value
    main.state.simulation = True
    main.state.connected = True
    main.state.motion_state = main.MotionState.IDLE
    main.state.clear_error()
    main.state.tool_state = "unknown"

    asyncio.run(
        main.execute_program_sequence(
            {
                "settings": {},
                "trajectory": {
                    "execution_steps": [
                        {
                            "kind": "tool",
                            "label": "Close gripper",
                            "tool": "gripper",
                            "action": "close",
                            "duration_s": 0.0,
                        }
                    ]
                },
            }
        )
    )

    tool_state = main.state.tool_state
    diagnostics = dict(main.state.motion_diagnostics)
    main.state.tool_state = previous_tool_state
    main.state.tool_value = previous_tool_value

    assert tool_state == "closed"
    assert diagnostics["result"] == "reached"
    assert diagnostics["active_step_index"] == 1


def test_program_execute_resumes_after_stop_and_reports_queued_state(monkeypatch):
    main.cancel_motion_tasks()
    preview_id = "program-resume-after-stop"
    observed_motion_states = []
    previous_motion_state = main.state.motion_state
    previous_execution_state = main.state.motion_execution_state
    previous_diagnostics = dict(main.state.motion_diagnostics)
    previous_pending_motion = dict(main.state.pending_motion)
    previous_connected = main.state.connected
    previous_simulation = main.state.simulation
    previous_last_command = main.state.last_command
    previous_last_error = main.state.last_error

    async def hold_program_open(_preview):
        observed_motion_states.append(main.state.motion_state)
        await asyncio.Event().wait()

    monkeypatch.setattr(main, "execute_program_sequence", hold_program_open)
    main.state.connected = True
    main.state.simulation = True
    main.state.motion_state = main.MotionState.STOPPED
    main.state.motion_execution_state = "stopped"
    main.state.motion_diagnostics = {"result": "stopped", "execution_state": "stopped"}
    main.path_previews[preview_id] = {
        "id": preview_id,
        "mode": "program",
        "program_revision": 9,
        "source": "program",
        **main.pose_snapshot_fields(),
        "trajectory": {
            "mode": "program",
            "duration_s": 1.0,
            "waypoint_count": 1,
            "waypoints": [list(main.state.reported_angles_deg)],
            "execution_steps": [{"kind": "motion", "label": "Move"}],
        },
    }

    async def run_test():
        try:
            response = await main.execute_path(
                main.PathExecuteRequest(preview_id=preview_id, program_revision=9)
            )
            assert response["ok"], response
            assert response["state"]["motion_state"] == "idle"
            assert response["state"]["motion_execution_state"] == "queued"
            assert response["state"]["motion_diagnostics"]["result"] == "queued"
            await asyncio.sleep(0)
            assert observed_motion_states == [main.MotionState.IDLE]
        finally:
            main.cancel_motion_tasks()
            await asyncio.sleep(0)

    try:
        asyncio.run(run_test())
    finally:
        main.path_previews.pop(preview_id, None)
        main.state.motion_state = previous_motion_state
        main.state.motion_execution_state = previous_execution_state
        main.state.motion_diagnostics = previous_diagnostics
        main.state.pending_motion = previous_pending_motion
        main.state.connected = previous_connected
        main.state.simulation = previous_simulation
        main.state.last_command = previous_last_command
        main.state.last_error = previous_last_error


def test_program_sequence_reports_missing_trajectory_as_failed():
    main.cancel_motion_tasks()
    previous_motion_state = main.state.motion_state
    previous_execution_state = main.state.motion_execution_state
    previous_diagnostics = dict(main.state.motion_diagnostics)
    previous_pending_motion = dict(main.state.pending_motion)
    previous_simulation = main.state.simulation
    previous_last_error = main.state.last_error
    main.state.simulation = True
    main.state.motion_state = main.MotionState.IDLE
    main.start_motion_diagnostics(
        source="program",
        mode="program",
        target_deg=list(main.state.reported_angles_deg),
        expected_duration_s=1.0,
        waypoint_count=0,
        step_total=1,
    )

    try:
        asyncio.run(
            main.execute_program_sequence(
                {
                    "trajectory": {
                        "execution_steps": [{"kind": "motion", "label": "Broken move"}],
                    }
                }
            )
        )
        assert main.state.motion_execution_state == "failed"
        assert main.state.motion_diagnostics["result"] == "failed"
        assert "missing its planned trajectory" in main.state.motion_diagnostics["error"]
    finally:
        main.cancel_motion_tasks()
        main.state.motion_state = previous_motion_state
        main.state.motion_execution_state = previous_execution_state
        main.state.motion_diagnostics = previous_diagnostics
        main.state.pending_motion = previous_pending_motion
        main.state.simulation = previous_simulation
        main.state.last_error = previous_last_error


def test_saved_program_plan_can_be_restored_without_replanning(monkeypatch, tmp_path):
    main.cancel_motion_tasks()
    main.path_previews.clear()
    main.state.connected = True
    main.state.simulation = True
    main.state.clear_error()
    monkeypatch.setattr(
        program_library,
        "PROGRAM_STORE_PATH",
        tmp_path / "programs.local.json",
    )
    client = TestClient(main.app)

    created = client.post(
        "/api/programs",
        json={
            "name": "Cached home",
            "steps": [
                {
                    "label": "Current pose",
                    "type": "joint",
                    "angles_deg": main.state.reported_angles_deg,
                }
            ],
        },
    ).json()
    assert created["ok"], created
    program = created["program"]

    planned = client.post(
        "/api/path/preview",
        json={
            "mode": "program",
            "program_id": program["id"],
            "program_revision": 4,
            "waypoints": program["steps"],
        },
    ).json()
    assert planned["ok"], planned
    assert planned["plan_cache"]["saved"]

    main.path_previews.clear()
    restored = client.post(
        f"/api/programs/{program['id']}/restore-plan",
        json={"program_revision": 5},
    ).json()

    assert restored["ok"], restored
    assert restored["restored"]
    assert restored["preview"]["program_revision"] == 5
    assert restored["preview"]["source"] == "saved_program_plan"
    assert restored["preview_id"] in main.path_previews

    original_angles = list(main.state.reported_angles_deg)
    try:
        main.state.reported_angles_deg[0] += 0.2
        stale = client.post(
            f"/api/programs/{program['id']}/restore-plan",
            json={"program_revision": 6},
        ).json()
    finally:
        main.state.reported_angles_deg = original_angles
        main.state.fk = main.forward_kinematics(original_angles, main.config.links)
        main.limiter.reset(original_angles)

    assert not stale["ok"]
    assert stale["cache_miss"]
    assert "start pose is stale" in stale["error"]
