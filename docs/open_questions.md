# Open Questions

This file tracks questions that are intentionally unresolved.

At this stage, unresolved questions are expected and useful. They show where the design is still flexible.

## System Split

- What logic should run on the PC versus the embedded controller?
- Should the embedded controller accept high-level move commands, joint targets, or low-level actuator commands?
- Is wireless control acceptable for runtime operation, or should it be limited to setup and maintenance?

## Robot Modeling

- What are the final joint definitions and axis conventions?
- What units should be used consistently across the project?
- How should link geometry and tool offsets be represented?
- How should joint limits, home offsets, and calibration data be stored?

## Vision

- Will each task need its own YOLO model or only different classes/configurations?
- What perception output should the rest of the system consume: image coordinates, robot-frame coordinates, poses, or something else?
- What level of confidence filtering and verification is needed before motion begins?

## Task Layer

- Should tasks be separate programs, plugins, or modes?
- How much of a task should be declarative configuration versus custom code?
- How should a task describe required tools, detections, and motion goals?

## Motion

- Where should inverse kinematics live?
- Where should path planning live?
- How advanced does path planning need to be for the project scope?
- How should failures such as unreachable targets or ambiguous detections be handled?

## GUI And Operations

- Should the GUI be mainly manual control, or also a task launcher and monitoring panel?
- What tuning and calibration workflows need GUI support?
- What live telemetry is actually useful to operators during testing?

## Hardware And Safety

- How will homing be performed?
- What recovery strategy is needed for open-loop stepper position drift?
- What fault conditions should stop the robot immediately?
- How should different end effectors be abstracted and controlled?
