# DynaMOE Team 19889 - FTC Robot Code

## Overview

Robot code for DynaMOE Team 19889, FIRST Tech Challenge DECODE 2025-26 season. Uses modular subsystem architecture shared between Autonomous and TeleOp. Autonomous is built on the **Ivy** command framework (`com.pedropathing:ivy`) for Pedro Pathing.

## Directory Structure

```
teamcode/
├── opmodes/           # Competition OpModes
│   ├── DynaMOE_19889_Auton.java    # Main autonomous
│   ├── DynaMOE_19889_TeleOp.java   # Driver control
│   ├── LauncherVelocityTuner.java  # Tuning utility
│   └── [test/legacy OpModes]
├── robot/             # RobotHardware.java - aggregates all subsystems
├── subsystems/        # Drivetrain, Launcher, Intake, ArtifactManager, LauncherAssist
├── util/              # RobotEnums, FieldPositions, MotifDetector, RobotLogger, RobotState
├── pedroPathing/      # Autonomous navigation
└── states/            # AprilTag vision
```

## Hardware Config Names

Must match Driver Hub configuration:

**Motors:** `leftFront`, `rightFront`, `leftBack`, `rightBack`, `leftLauncher` (DcMotorEx), `rightLauncher` (DcMotorEx), `intakeMotor`

**Servos:** `leftFeeder` (CRServo), `rightFeeder` (CRServo)

## Quick Start

```java
RobotHardware robot = new RobotHardware(telemetry);
robot.init(hardwareMap);

robot.drivetrain.setPowers(1, 1, 1, 1);
robot.launcher.spinUp(true);
robot.intake.intake();

robot.stopAllSubsystems();
```

## Competition OpModes

### Autonomous (Ivy Command-Based)
D-Pad selects position: UP=Blue Goal, DOWN=Blue Perimeter, LEFT=Red Goal, RIGHT=Red Perimeter

Uses Ivy's `Scheduler` with composable commands (`sequential`, `parallel`, `race`, `follow`, `instant`, `waitMs`, `waitUntil`). All paths are pre-built during init. Background `infinite` commands handle `follower.update()`, subsystem updates, and pose saving every tick.

Pose is continuously saved for TeleOp via `RobotState`.

### TeleOp Controls

**Single-Button Launch (Primary):**
- **RB: Start launch sequence** (auto-align + auto-velocity + fire both sides)
- B: Abort launch sequence

**Drive & Intake:**
- Left Stick: Drive
- Right Stick: Rotate (disabled during auto-align)
- RT: Intake, LT: Outtake
- D-Pad L/R: Toggle field/robot centric

**Manual Launcher (Backup):**
- A: Spin up manually
- X: Feed Left, Y: Feed Right
- Gamepad2 D-Pad U/D: Adjust manual speed ±50 RPM

## Auto-Velocity Table (Tuned 2026-02-03)

| Distance | RPM |
|----------|-----|
| 60" | 1090 |
| 70" | 1170 |
| 80-90" | 1180 |
| 101" | 1230 |
| 120" | 1300 |
| 143" | 1420 |
| 150" | 1490 |

## Tuning

**Launcher velocities** in `Launcher.java`: CLOSE=1150 RPM, FAR=1520 RPM

**Auto-velocity table** in `LauncherAssist.java`: Distance → RPM mapping

**Field positions** in `FieldPositions.java`

**Pose transfer** via `RobotState.java`: Auton end pose → TeleOp start pose

## Team Info
**Team:** DynaMOE 19889 | **Season:** DECODE 2025-26

**Last Updated:** 2026-02-14
