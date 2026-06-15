from pathlib import Path
from shutil import copyfile

from app import config as config_module


def test_load_config_prefers_local_config(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[1] / "config" / "robot.example.yaml"
    example = tmp_path / "robot.example.yaml"
    local = tmp_path / "robot.local.yaml"
    copyfile(source, example)
    copyfile(source, local)
    text = local.read_text(encoding="utf-8").replace("base_height: 160.0", "base_height: 123.0")
    local.write_text(text, encoding="utf-8")
    monkeypatch.setattr(config_module, "EXAMPLE_CONFIG_PATH", example)
    monkeypatch.setattr(config_module, "LOCAL_CONFIG_PATH", local)

    loaded = config_module.load_config()

    assert loaded.source_path == local
    assert loaded.links.base_height_mm == 123.0


def test_ensure_local_config_copies_example(tmp_path, monkeypatch):
    source = Path(__file__).resolve().parents[1] / "config" / "robot.example.yaml"
    example = tmp_path / "robot.example.yaml"
    local = tmp_path / "robot.local.yaml"
    copyfile(source, example)
    monkeypatch.setattr(config_module, "EXAMPLE_CONFIG_PATH", example)
    monkeypatch.setattr(config_module, "LOCAL_CONFIG_PATH", local)

    result = config_module.ensure_local_config()

    assert result == local
    assert local.exists()
    assert "simulation_default" in local.read_text(encoding="utf-8")
