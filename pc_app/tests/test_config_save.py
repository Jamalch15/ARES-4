from pathlib import Path
from shutil import copyfile

from app.config import load_config, save_calibration_updates


def test_save_calibration_updates_values_and_preserves_comments(tmp_path):
    source = Path(__file__).resolve().parents[1] / "config" / "robot.example.yaml"
    target = tmp_path / "robot.yaml"
    copyfile(source, target)

    save_calibration_updates(
        target,
        {
            "links_mm": {"base_height": 81.5},
            "joints": [
                {
                    "limits_deg": {"min": -150.0, "max": 150.0},
                    "home_deg": 1.0,
                    "max_speed_deg_s": 44.0,
                    "max_accel_deg_s2": 111.0,
                    "zero_offset_deg": 2.0,
                    "direction_sign": -1,
                    "hardware": {
                        "stepper": {
                            "enabled": True,
                            "step_pin": 17,
                            "dir_pin": 16,
                            "enable_pin": -1,
                            "enable_active_low": True,
                            "m0_pin": 3,
                            "m1_pin": 8,
                            "m2_pin": 18,
                            "driver_model": "DRV8825",
                            "motor_full_steps_per_rev": 200,
                            "microsteps": 32,
                            "gear_ratio": 4.5,
                        }
                    },
                }
            ],
            "motion": {"command_rate_limit_hz": 14.0, "acceleration_deg_s2": 130.0},
        },
    )

    text = target.read_text(encoding="utf-8")
    saved = load_config(target)

    assert "# Placeholder measurements" in text
    assert saved.links.base_height_mm == 81.5
    assert saved.joints[0].min_deg == -150.0
    assert saved.joints[0].max_deg == 150.0
    assert saved.joints[0].home_deg == 1.0
    assert saved.joints[0].max_speed_deg_s == 44.0
    assert saved.joints[0].max_accel_deg_s2 == 111.0
    assert saved.joints[0].zero_offset_deg == 2.0
    assert saved.joints[0].direction_sign == -1
    assert saved.joints[0].hardware.stepper.enabled is True
    assert saved.joints[0].hardware.stepper.step_pin == 17
    assert saved.joints[0].hardware.stepper.dir_pin == 16
    assert saved.joints[0].hardware.stepper.microsteps == 32
    assert saved.joints[0].hardware.stepper.gear_ratio == 4.5
    assert saved.motion.command_rate_limit_hz == 14.0
    assert saved.motion.acceleration_deg_s2 == 130.0
