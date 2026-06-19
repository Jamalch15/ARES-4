# AGENTS.md

## Purpose

This file gives coding agents a safe starting point for working in this repository.

The project definition is still fluid. Agents should treat all architecture notes, module boundaries, and workflow suggestions as provisional unless a later document or direct user instruction makes them concrete.

## Project State

- This repository is in an early planning stage
- The system design is not finalized
- Hardware details may change
- Task definitions may change
- Folder structure may change
- Interface boundaries may change

Agents should optimize for clarity and low-friction iteration, not premature rigidity.

Agents should also keep changes future-oriented: prefer small interfaces and
data shapes that can support the planned robot stack later, and avoid quick
fixes that make future motion planning, perception, transport, calibration, or
hardware changes harder.

## Early Project Direction

The broad idea is a software stack for a 4DOF robot arm with:

- Robot motion logic
- Vision-based perception
- Task-specific programs or modules
- PC-to-controller communication
- GUI-based operation and tuning

The current working assumption is that the PC handles heavier computation while the embedded controller handles low-level control. This is an assumption, not a final rule.

## Documentation Rules

When adding or updating documentation:

- Clearly label assumptions as assumptions
- Avoid presenting early ideas as fixed design decisions
- Prefer phrases like "current idea", "working assumption", and "not yet decided" where appropriate
- Capture open questions instead of hiding them
- Keep docs short enough to stay maintainable

## Coding Rules For Early Development

When writing code or proposing structure:

- Prefer modular boundaries that can be changed later
- Avoid deeply coupling task logic to hardware-specific control
- Avoid locking the project into one vision model, one transport layer, or one GUI framework too early
- Prefer configuration files for values that are likely to change
- Document units, coordinate frames, and hardware assumptions explicitly whenever they appear
- When fixing a near-term issue, consider the likely future subsystem it will
  belong to and leave a path toward that subsystem instead of embedding a
  one-off shortcut

## Likely Subsystems

These are useful mental buckets, not final package names:

- Vision or perception
- Task logic
- IK and path planning
- Robot interface or command layer
- Embedded motion control
- Calibration and configuration
- GUI or operator tools

## When In Doubt

If the repo contains an unfinished or ambiguous design:

- Preserve flexibility
- Add notes instead of inventing certainty
- Surface tradeoffs plainly
- Keep interfaces simple

The main goal at this stage is to support exploration while keeping the project understandable.

## Tests And GitHub Actions

GitHub Actions runs the PC application tests on Ubuntu from a clean checkout.
The developer machine may also contain the ignored
`pc_app/config/robot.local.yaml`, so a test that calls `load_config()` can
silently use different robot geometry or calibration locally than it uses in
CI.

The June 18, 2026 failure on commit `0476bef` was caused by this difference.
`test_endpoint_ik_escape_restores_inward_authority_at_exact_extension` passed
against the local configuration but failed in GitHub Actions because the clean
checkout used `robot.example.yaml`. The behavior was valid in both cases, but
the test required one incidental solver mode string.

To prevent this:

- Tests must use `load_config(EXAMPLE_CONFIG_PATH)` when they require the
  committed reference robot configuration.
- Use bare `load_config()` only when the purpose of the test is specifically
  to exercise local-config selection or normal runtime configuration loading.
- Assert the externally relevant behavior, coordinates, limits, or safety
  result. Do not require one internal solver/fallback label when multiple
  solver paths satisfy the same contract.
- Before pushing, run the complete suite from `pc_app` with:
  `python -m pytest`.
- When a change may depend on ignored local files, also test from a clean
  checkout or a copied tree that excludes `robot.local.yaml`.
- Do not commit localhost server logs, pytest caches, virtual environments, or
  local robot calibration files.
- After pushing, wait for the `Python tests` GitHub Actions workflow and verify
  that it completes successfully. A successful local run is not sufficient.
