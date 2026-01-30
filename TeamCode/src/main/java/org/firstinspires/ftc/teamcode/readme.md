# DynaMOE Team 19889 - FTC Robot Code

## Overview

Robot code for DynaMOE Team 19889, FIRST Tech Challenge DECODE 2025-26 season. Uses modular subsystem architecture shared between Autonomous and TeleOp.

## Directory Structure

```
teamcode/
├── opmodes/           # Competition OpModes
│   ├── DynaMOE_19889_Auton.java    # Main autonomous
│   ├── DynaMOE_19889_TeleOp.java   # Driver control
│   └── [test/legacy OpModes]
├── robot/             # RobotHardware.java - aggregates all subsystems
├── subsystems/        # Drivetrain, Launcher, Intake, ArtifactManager, LauncherAssist
├── util/              # RobotEnums, FieldPositions, MotifDetector, RobotLogger
├── pedroPathing/      # Autonomous navigation
└── states/            # AprilTag vision
```

## Hardware Config Names

Must match Driver Hub configuration:

**Motors:** `leftFront`, `rightFront`, `leftBack`, `rightBack`, `leftLauncher` (DcMotorEx), `rightLauncher` (DcMotorEx), `intakeMotor`

**Servos:** `leftFeeder` (CRServo), `rightFeeder` (CRServo), `diverter` (Servo)

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

### Autonomous
D-Pad selects position: UP=Blue Goal, DOWN=Blue Perimeter, LEFT=Red Goal, RIGHT=Red Perimeter

### TeleOp Controls
- Left Stick: Drive, Right Stick: Rotate
- RT: Intake, LT: Outtake
- A: Launchers ON, B: OFF
- X: Feed Left, Y: Feed Right
- LB: Toggle auto-align, RB: Toggle auto-velocity
- D-Pad L/R: Toggle field/robot centric

## Tuning

**Launcher velocities** in `Launcher.java`: CLOSE=1150 RPM, FAR=1520 RPM

**Field positions** in `FieldPositions.java`

**Team:** DynaMOE 19889 | **Season:** DECODE 2025-26
