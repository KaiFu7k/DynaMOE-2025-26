# DynaMOE Team 19889 - FTC Robot Code

## Overview

This is the robot code for DynaMOE Team 19889 competing in the FIRST Tech Challenge DECODE game (2024-25 season). The codebase uses a modular subsystem architecture that allows code sharing between Autonomous and TeleOp modes.

## Code Architecture

### Philosophy
Our code is organized into **subsystems** - independent modules that each control a specific part of the robot. This makes the code:
- **Easier to test** - Test each subsystem independently
- **Easier to debug** - Problems are isolated to specific subsystems
- **Reusable** - Same code works in both Autonomous and TeleOp
- **Maintainable** - Changes in one place affect all modes

### Directory Structure

```
teamcode/
├── opmodes/                  # Competition OpModes (what you select on Driver Hub)
│   ├── DynaMOE_19889_Auton.java              # Original autonomous (kept for backup)
│   ├── DynaMOE_19889_Auton_Refactored.java   # New modular autonomous ⭐
│   ├── DynaMOE_19889_TeleOp.java             # Driver-controlled mode ⭐
│   ├── FieldCentricMecanumDrive.java         # Field-centric drive (legacy)
│   ├── RobotCentricMecanumDrive.java         # Robot-centric drive (legacy)
│   └── [other test OpModes...]
│
├── robot/                    # Main robot class
│   └── RobotHardware.java                    # Aggregates all subsystems ⭐
│
├── subsystems/               # Robot subsystems (NEW)
│   ├── Drivetrain.java                       # Mecanum drive control
│   ├── Launcher.java                         # Launcher motors, feeders, diverter
│   ├── Intake.java                           # Intake motor
│   └── ArtifactManager.java                  # Artifact tracking & MOTIF logic
│
├── util/                     # Utility classes (NEW)
│   ├── RobotEnums.java                       # Shared enums
│   ├── FieldPositions.java                   # Field coordinates & positions
│   ├── MotifDetector.java                    # MOTIF pattern detection
│   └── RobotLogger.java                      # Structured logging
│
├── pedroPathing/             # Pedro Pathing library integration
│   └── Constants.java                        # Autonomous navigation constants
│
└── states/                   # State machines & vision
    ├── ReadAprilTag.java
    └── AprilTagRunTest.java
```

## Hardware Configuration Names

**CRITICAL:** These names must match your Driver Hub configuration!

### Motors
- `left_front_drive`, `right_front_drive`, `left_back_drive`, `right_back_drive`
- `left_launcher` (DcMotorEx), `right_launcher` (DcMotorEx)
- `intake`

### Servos
- `left_feeder` (CRServo), `right_feeder` (CRServo), `diverter` (Servo)

## Quick Start

### Using the Robot in OpModes

```java
// Initialize robot
RobotHardware robot = new RobotHardware(telemetry);
robot.init(hardwareMap);

// Use subsystems
robot.drivetrain.setPowers(1, 1, 1, 1);
robot.launcher.spinUp(true);
robot.intake.intake();

// Clean up
robot.stopAllSubsystems();
```

## Subsystems Reference

### Drivetrain
```java
robot.drivetrain.setPowers(lf, rf, lb, rb);
robot.drivetrain.stop();
robot.drivetrain.setBrakeMode(true);
```

### Launcher
```java
robot.launcher.spinUp(true);  // true=close, false=far
robot.launcher.isReady();
robot.launcher.feed(LauncherSide.LEFT, this::opModeIsActive);
robot.launcher.stop();
```

### Intake
```java
robot.intake.intake();
robot.intake.outtake();
robot.intake.stop();
```

### ArtifactManager
```java
robot.artifactManager.configureDefaultPreload();
robot.artifactManager.findArtifactSide(ArtifactColor.PURPLE);
robot.artifactManager.getPatternSequence(Motif.GPP);
```

## Competition OpModes

### Autonomous: DynaMOE_19889_Auton_Refactored
**Match Day Steps:**
1. Place robot at starting position
2. Select "DynaMOE 19889 Auto [Refactored]"
3. D-Pad: Select position (UP=Blue Goal, DOWN=Blue Perimeter, LEFT=Red Goal, RIGHT=Red Perimeter)
4. Y: Cycle MOTIF pattern
5. A: Confirm
6. START

### TeleOp: DynaMOE_19889_TeleOp
**Gamepad 1 Controls:**
- Left Stick: Drive, Right Stick: Rotate
- RT: Intake, LT: Outtake
- A: Launchers ON, B: OFF
- X: Feed Left, Y: Feed Right
- D-Pad L/R: Toggle drive mode

## Tuning Constants

### Field Positions
Edit `util/FieldPositions.java`

### Launcher Velocities
Edit `subsystems/Launcher.java`:
- LAUNCHER_CLOSE_TARGET_VELOCITY = 1200 RPM
- LAUNCHER_FAR_TARGET_VELOCITY = 1350 RPM

### Artifact Preload
Default: LEFT=[Purple, Purple], RIGHT=[Green]

## Debugging

```java
// Enable debug logging
robot.logger.setMinLogLevel(RobotLogger.LogLevel.DEBUG);

// View telemetry
robot.updateTelemetry();
```

## Team Info
**Team:** DynaMOE 19889  
**Season:** 2024-25 (DECODE)  
**License:** MIT

Good luck at competition! 🤖🏆
