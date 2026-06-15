# Modular 4DOF Arm

## Status

This repository is at an early concept stage.

The ideas described here are intentionally vague, incomplete, and subject to change. Nothing in this document set should be treated as a final architecture, final feature list, or final hardware/software split. The purpose of these docs is to capture the current direction of the project so development can start from a shared mental model.

## Early Project Overview

The project is a modular 4DOF robot arm for a mechatronics project.

At a high level, the software side is expected to cover:

- Robot motion logic such as inverse kinematics, motion sequencing, and path planning
- Vision processing, likely including YOLO or related detection pipelines
- Task-specific application logic for different robot tasks
- Communication between a host computer and an embedded controller
- A GUI for operating, tuning, calibrating, and testing the robot

## Current Working Idea

The current idea is to split computation between:

- PC side
- Embedded controller side

The rough expectation is:

- The PC handles heavy computation such as vision, task reasoning, and possibly IK/path planning
- The embedded controller handles low-level actuator control and hardware-facing behavior

This is only a working assumption. It may change once timing, reliability, and integration constraints are better understood.

## Robot Concept

The robot arm is currently expected to include:

- Two open-loop stepper-driven joints
- Two hobby-servo-driven joints
- One interchangeable end effector depending on task

Possible end effectors may include:

- Gripper
- Electromagnet
- Other task-specific tooling

The exact actuator arrangement, joint roles, coordinate conventions, and tool control interfaces are not final.

## Task-Oriented Direction

The robot is expected to perform multiple tasks. The current idea is that each task may have:

- Its own perception setup
- Its own task logic
- Its own object classes or detection targets
- Shared robot motion and control infrastructure underneath

Examples of future task types could include:

- Pick and place
- Sorting
- Object-specific handling
- Vision-guided interaction with a workspace

It is not yet decided whether each task should be a separate program, a plugin/module, or a configuration-driven mode inside one larger application.

## Documentation Map

- [AGENTS.md](AGENTS.md): guidance for coding agents working in this repository
- [pc_app/README.md](pc_app/README.md): local PC operator dashboard
- [controller_firmware/README.md](controller_firmware/README.md): ESP32-S3 firmware notes and PlatformIO commands
- `electronics/`: KiCad electronics design files
- [docs/architecture.md](docs/architecture.md): rough system architecture ideas
- [docs/component_master_plan.md](docs/component_master_plan.md): broad component planning guide
- [docs/accelerated_exam_demo_plan.md](docs/accelerated_exam_demo_plan.md): faster exam-focused demo roadmap
- [docs/tasks/README.md](docs/tasks/README.md): early notes on task structure
- [docs/open_questions.md](docs/open_questions.md): unresolved decisions and design questions

## GitHub And Local Files

The intended GitHub repository name is `modular-4dof-arm`.

Tracked configuration should stay generic. Machine-specific calibration and
hardware values belong in `pc_app/config/robot.local.yaml`, which is ignored by
Git. The tracked `pc_app/config/robot.example.yaml` remains the template and
simulation-safe fallback.

## Intended Use Of These Docs

These docs are here to:

- Capture current intent before details are forgotten
- Give future contributors and agents a shared starting point
- Make assumptions explicit
- Leave room for the design to evolve without pretending decisions are final

If a future implementation conflicts with these notes, the implementation or updated design discussion should take priority over outdated assumptions in these early docs.
