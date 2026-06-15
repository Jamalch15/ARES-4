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
