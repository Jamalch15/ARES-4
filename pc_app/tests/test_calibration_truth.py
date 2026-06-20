import subprocess
import sys
from pathlib import Path

from pytest import approx

from app.calibration_truth import calibration_pose_report, model_truth_summary
from app.config import EXAMPLE_CONFIG_PATH, load_config
from app.kinematics import forward_kinematics


def test_model_truth_summary_declares_chain_and_mapping_split():
    config = load_config(EXAMPLE_CONFIG_PATH)

    summary = model_truth_summary(config)

    assert [step["id"] for step in summary["transform_chain"]] == [
        "actuator",
        "logical_joint",
        "dh_theta",
        "flange",
        "tcp",
        "command_correction",
    ]
    first_joint = summary["joint_conventions"][0]
    assert first_joint["actuator_mapping"]["role"] == "controller actuator-to-logical-joint mapping"
    assert first_joint["dh_model_mapping"]["role"] == "logical joint q to Standard-DH theta mapping"
    assert summary["active_tool"]["tcp_axis_mapping"]["tool_z"] == "local DH +X / tool-forward"
    assert len(summary["renderer_parity_cases"]) == 3


def test_calibration_pose_report_computes_tcp_residuals():
    config = load_config(EXAMPLE_CONFIG_PATH)
    fk = forward_kinematics(config.home_pose, config.links)

    report = calibration_pose_report(
        config,
        [
            {
                "id": "home_touch",
                "angles_deg": config.home_pose,
                "measured_tcp_mm": {
                    "x_mm": fk["x_mm"],
                    "y_mm": fk["y_mm"],
                    "z_mm": fk["z_mm"] - 25.0,
                },
                "measurement_method": "test fixture",
            }
        ],
    )

    row = report["rows"][0]
    assert row["ok"]
    assert row["expected_fk_tcp_mm"]["z_mm"] == approx(fk["z_mm"])
    assert row["residual_tcp_mm"]["z_mm"] == approx(-25.0)


def test_calibration_truth_report_cli_prints_markdown():
    pc_app = Path(__file__).resolve().parents[1]

    result = subprocess.run(
        [
            sys.executable,
            "tools/calibration_truth_report.py",
            "--config",
            "config/robot.example.yaml",
        ],
        cwd=pc_app,
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )

    assert "# Calibration Truth Report" in result.stdout
    assert "Transform Chain" in result.stdout
    assert "Joint Convention Split" in result.stdout
    assert "Private measurement files should stay outside git" in result.stdout
