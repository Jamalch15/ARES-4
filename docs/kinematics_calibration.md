# Cartesian TCP calibration

## Current design

The current implementation uses a tool-specific Cartesian command correction.
It does not change FK, the DH model, joint conventions, or camera/workspace
calibration.

All coordinates are robot-base-frame millimetres:

- +X is sideways.
- +Y is forward.
- +Z is upward.
- Tool pitch remains in degrees and is not corrected by the first model.

For an intended physical TCP target, the correction computes the model target
sent to IK. Joint-space commands and Cartesian velocity jogging are unchanged.
Endpoint previews, programs, named Cartesian positions, task targets, and
absolute live Cartesian targets share this command-layer path.

## Configuration

Results are stored under `kinematics_calibration`. Profiles are keyed by the
active tool. A profile records:

- workspace-map signature and tool name;
- fit and validation samples;
- fitted coefficients and model type;
- rejected outlier IDs;
- before/after metrics and diagnostic notes;
- independent global/profile enable flags.

Absent or disabled settings are a no-op.

## Samples

Each sample stores:

- intended physical target;
- corrected or uncorrected model command target;
- reported joint angles and their source;
- FK-predicted TCP from those joints;
- measured TCP XYZ;
- model, command/IK, and final landing residuals;
- timestamp, measurement source, role, quality, and notes.

The fit uses `measured - FK(reported joints)`. This avoids treating normal IK
target error as robot-model error. Validation landing error separately compares
the measured TCP with the intended physical target.

Camera XY and manual/touch-off Z remain measurements, not ground truth. The UI
does not perform automatic touch-off motion; it only computes measured Z from a
known surface height plus an operator-entered TCP contact offset.

## Models

- `constant_xyz`: constant XYZ offset; minimum two fit samples.
- `affine_xy_z_offset`: affine XY map plus constant Z offset; minimum four
  samples spanning at least three non-collinear XY positions.

Fitting uses sample-quality weights and iterative median-absolute-deviation
outlier rejection. A fitted affine transform is rejected if it is singular or
ill-conditioned.

The default pass/warn/fail thresholds are working assumptions:

- pass: XY RMSE <= 5 mm, XY max <= 10 mm, Z RMSE <= 3 mm, Z max <= 5 mm;
- warning: XY RMSE <= 8 mm, XY max <= 15 mm, Z RMSE <= 5 mm, Z max <= 8 mm;
- otherwise fail.

## Limitations and diagnostics

This correction can compensate repeatable Cartesian error. It cannot identify
one unique mechanical cause. Diagnostics flag patterns compatible with:

- consistent TCP or joint-zero offset;
- geometry or workspace scale/skew;
- IK/FK-to-command error;
- large Z reference/TCP length error;
- poor repeatability from backlash, compliance, or noisy measurement.

If the workspace calibration changes after fitting, previews warn about the
signature mismatch. Refit after moving the camera/workplate, changing the tool,
changing geometry/joint calibration, or observing direction-dependent backlash.
