from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

import yaml


PC_APP_DIR = Path(__file__).resolve().parents[1]
if str(PC_APP_DIR) not in sys.path:
    sys.path.insert(0, str(PC_APP_DIR))

from app.calibration_truth import calibration_pose_report, model_truth_summary  # noqa: E402
from app.config import load_config  # noqa: E402


def _load_measurements(path: Path | None) -> list[dict[str, Any]]:
    if path is None:
        return []
    with path.open("r", encoding="utf-8") as handle:
        if path.suffix.lower() == ".json":
            payload = json.load(handle)
        else:
            payload = yaml.safe_load(handle) or {}
    if isinstance(payload, list):
        return payload
    if isinstance(payload, dict) and isinstance(payload.get("poses"), list):
        return payload["poses"]
    raise ValueError("measurement file must be a list or a mapping with a poses list")


def _point_text(point: dict[str, Any] | None) -> str:
    if not point:
        return "-"
    return (
        f"x {float(point.get('x_mm', 0.0)):.2f}, "
        f"y {float(point.get('y_mm', 0.0)):.2f}, "
        f"z {float(point.get('z_mm', 0.0)):.2f}"
    )


def _markdown(summary: dict[str, Any], report: dict[str, Any], source: Path) -> str:
    lines: list[str] = []
    tool = summary["active_tool"]
    lines.append("# Calibration Truth Report")
    lines.append("")
    lines.append(f"- Config: `{source}`")
    lines.append(f"- Active geometry preset: `{summary['active_geometry_preset']}`")
    lines.append(
        f"- Active tool: `{tool['name']}` ({tool['type']}), TCP {_point_text(tool['tcp_offset_mm'])} mm"
    )
    lines.append(f"- Tool dimensions validated: `{tool['dimensions_validated']}`")
    lines.append("")
    lines.append("## Transform Chain")
    lines.append("")
    for index, step in enumerate(summary["transform_chain"], start=1):
        lines.append(f"{index}. **{step['label']}**")
        lines.append(f"   - {step['notes']}")
        if step.get("equation"):
            lines.append(f"   - `{step['equation']}`")
    lines.append("")
    lines.append("## Joint Convention Split")
    lines.append("")
    lines.append("| Joint | Actuator mapping | DH theta mapping | Home |")
    lines.append("|---|---|---|---|")
    for row in summary["joint_conventions"]:
        actuator = row["actuator_mapping"]
        dh = row["dh_model_mapping"]
        lines.append(
            f"| J{row['joint']} {row['name']} | "
            f"zero {actuator['zero_offset_deg']:.2f} deg, sign {actuator['direction_sign']:+d} | "
            f"theta = q*{dh['direction_sign']:+d} + {dh['zero_offset_deg']:.2f} + {dh['theta_offset_deg']:.2f} | "
            f"{row['mechanical_home_deg']:.2f} deg |"
        )
    lines.append("")
    lines.append("## Z Audit Order")
    lines.append("")
    for index, item in enumerate(summary["z_audit_order"], start=1):
        lines.append(f"{index}. {item}")
    lines.append("")
    lines.append("## Measurement Rows")
    lines.append("")
    if not report["rows"]:
        lines.append("No measurement file was supplied. Use `--measurements path.yaml` with a private local sheet.")
    else:
        lines.append("| Pose | Tool | FK TCP | Measured TCP | TCP residual | Method |")
        lines.append("|---|---|---|---|---|---|")
        for row in report["rows"]:
            if not row.get("ok"):
                lines.append(f"| {row['id']} | - | - | - | {row['error']} | - |")
                continue
            lines.append(
                f"| {row['id']} | {row['tool']} | "
                f"{_point_text(row['expected_fk_tcp_mm'])} | "
                f"{_point_text(row['measured_tcp_mm'])} | "
                f"{_point_text(row['residual_tcp_mm'])} | "
                f"{row['measurement_method'] or '-'} |"
            )
    lines.append("")
    lines.append("Private measurement files should stay outside git or use ignored local paths.")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Print DH/joint/TCP calibration truth diagnostics.")
    parser.add_argument("--config", type=Path, default=None, help="Config path. Defaults to normal load_config selection.")
    parser.add_argument("--measurements", type=Path, default=None, help="Private YAML/JSON measurement sheet.")
    parser.add_argument("--format", choices=["markdown", "json"], default="markdown")
    args = parser.parse_args()

    config = load_config(args.config)
    measurements = _load_measurements(args.measurements)
    summary = model_truth_summary(config)
    report = calibration_pose_report(config, measurements)
    if args.format == "json":
        print(json.dumps({"model_truth": summary, "calibration_report": report}, indent=2, sort_keys=True))
    else:
        print(_markdown(summary, report, config.source_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
