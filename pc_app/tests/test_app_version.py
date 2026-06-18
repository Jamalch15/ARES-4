from fastapi.testclient import TestClient

import app.main as main


def test_version_endpoint_reports_running_and_disk_build():
    client = TestClient(main.app)

    payload = client.get("/api/version").json()

    assert payload["ok"]
    assert payload["running_build_id"] == main.RUNNING_BUILD_ID
    assert payload["disk_build_id"] == main.source_fingerprint()
    assert payload["restart_required"] is False
    assert payload["started_at"]


def test_version_endpoint_reports_restart_when_files_change(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(main, "source_fingerprint", lambda: "newer-files12")
    monkeypatch.setattr(main, "_origin_main_revision", lambda: main.RUNNING_GIT["commit"])

    payload = client.get("/api/version").json()

    assert payload["running_build_id"] == main.RUNNING_BUILD_ID
    assert payload["disk_build_id"] == "newer-files12"
    assert payload["restart_required"] is True
    assert "source or configuration files changed" in payload["reasons"][0]


def test_version_endpoint_reports_checkout_behind_origin(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(main, "source_fingerprint", lambda: main.RUNNING_BUILD_ID)
    monkeypatch.setattr(
        main,
        "_git_revision",
        lambda: {"commit": main.RUNNING_GIT["commit"], "dirty": False},
    )
    monkeypatch.setattr(main, "_origin_main_revision", lambda: "abcdef123456")

    payload = client.get("/api/version").json()

    assert payload["restart_required"] is False
    assert payload["pull_required"] is True
    assert payload["origin_main_commit"] == "abcdef123456"
    assert "not at origin/main" in payload["reasons"][0]


def test_version_endpoint_reports_server_started_before_checkout_changed(monkeypatch):
    client = TestClient(main.app)
    monkeypatch.setattr(main, "source_fingerprint", lambda: main.RUNNING_BUILD_ID)
    monkeypatch.setattr(
        main,
        "_git_revision",
        lambda: {"commit": "123456abcdef", "dirty": False},
    )
    monkeypatch.setattr(main, "_origin_main_revision", lambda: "123456abcdef")

    payload = client.get("/api/version").json()

    assert payload["restart_required"] is True
    assert payload["pull_required"] is False
    assert "checkout changed after the server started" in payload["reasons"][0]


def test_index_injects_running_build_and_disables_cache():
    client = TestClient(main.app)

    response = client.get("/")

    assert response.status_code == 200
    assert "__APP_BUILD_ID__" not in response.text
    assert f'name="app-build-id" content="{main.RUNNING_BUILD_ID}"' in response.text
    assert f"/static/app.js?v={main.RUNNING_BUILD_ID}" in response.text
    assert "no-store" in response.headers["cache-control"]
