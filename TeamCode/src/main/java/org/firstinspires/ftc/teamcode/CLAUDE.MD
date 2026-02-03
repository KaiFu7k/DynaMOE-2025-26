# CLAUDE.MD - AI Assistant Guide

Context for AI assistants working with DynaMOE Team 19889 FTC robot code.

## Project Info
- **Team:** DynaMOE 19889 | **Season:** DECODE 2025-26
- **Language:** Java | **Framework:** FTC SDK
- **Robot:** Mecanum drive with dual launcher

## Architecture

Modular subsystem-based. Subsystems are independent and shared between Auton/TeleOp.

```
opmodes/       - OpModes (DynaMOE_19889_Auton, DynaMOE_19889_TeleOp, LauncherVelocityTuner)
robot/         - RobotHardware.java (aggregates subsystems)
subsystems/    - Drivetrain, Launcher, Intake, ArtifactManager, LauncherAssist
util/          - RobotEnums, FieldPositions, MotifDetector, RobotLogger, RobotState
pedroPathing/  - Autonomous navigation
states/        - AprilTag vision
```

## Hardware Config Names

**Must match Driver Hub exactly:**

| Type | Names |
|------|-------|
| Drive motors | `leftFront`, `rightFront`, `leftBack`, `rightBack` |
| Launcher motors | `leftLauncher`, `rightLauncher` (DcMotorEx) |
| Intake | `intakeMotor` |
| Feeders | `leftFeeder`, `rightFeeder` (CRServo) |
| Diverter | `diverter` (Servo) |

## Tuning Constants

| What | Where | Current Values |
|------|-------|----------------|
| Launcher velocity | `Launcher.java` | Close: 1150 RPM, Far: 1520 RPM |
| Auto-velocity table | `LauncherAssist.java` | See table below (tuned 2026-02-03) |
| Field positions | `FieldPositions.java` | See file for coordinates |
| PID constants | `LauncherAssist.java` | Kp=1.5, Kd=0.1 |

### Auto-Velocity Table (Tuned)

| Distance (in) | Velocity (RPM) |
|---------------|----------------|
| 60 | 1090 |
| 70 | 1170 |
| 80-90 | 1180 (plateau) |
| 101 | 1230 |
| 113 | 1250 |
| 120 | 1300 |
| 130 | 1320 |
| 143 | 1420 |
| 150 | 1490 |
| 157 | 1480 |

## TeleOp Controls

### Single-Button Launch Mode
| Button | Action |
|--------|--------|
| **Right Bumper** | Start launch sequence (auto-align + auto-velocity + fire both sides) |
| **B** | Abort launch sequence |

### Drive & Intake
| Button | Action |
|--------|--------|
| Left Stick | Drive |
| Right Stick | Rotate (disabled during auto-align) |
| Right Trigger | Intake |
| Left Trigger | Outtake |
| D-pad L/R | Toggle robot/field centric |

### Manual Launcher (Backup)
| Button | Action |
|--------|--------|
| A | Manual spin up |
| X | Manual feed LEFT |
| Y | Manual feed RIGHT |
| Gamepad2 D-pad U/D | Adjust manual speed ±50 RPM |

## Auton → TeleOp Pose Transfer

Robot pose is preserved between Auton and TeleOp via `RobotState.java`:
- Auton saves final pose: `RobotState.saveAutonEndPose(pose, alliance)`
- TeleOp reads it: `RobotState.getAutonEndPose()`
- Pose expires after 5 minutes
- If no valid pose, TeleOp falls back to manual selection

## Field Coordinates

Origin (0,0) at bottom-left. X: 0-144 (left-right), Y: 0-144 (bottom-top).
Heading: 0°=right, 90°=up, 180°=left, 270°=down.

| Position | X | Y | Heading |
|----------|---|---|---------|
| Blue Goal Start | 36 | 132 | 90° |
| Red Goal Start | 108 | 132 | 90° |
| Blue Perimeter | 48 | 9 | 90° |
| Red Perimeter | 96 | 9 | 90° |
| Blue Launch | 48 | 96 | 135° |
| Red Launch | 96 | 96 | 45° |

## MOTIF Patterns
GPP, PGP, PPG (G=Green, P=Purple) - order artifacts must be launched.

## Common Issues

| Problem | Check |
|---------|-------|
| Motors not moving | Hardware config names, motor directions |
| Launcher slow | Battery voltage, DcMotorEx vs DcMotor |
| Drive wrong | IMU calibration, motor directions |
| Auto path off | FieldPositions, starting position selection |
| Wrong velocity | Check distance calculation, verify pose is set correctly |
| Accidental launch | Ensure `launcher.stop()` called before intake |

## AI Guidelines

1. **Preserve working code** - Don't modify competition OpModes without permission
2. **Test first** - Create test OpModes for new features
3. **Hardware safety** - No excessive speeds or conflicting commands
4. **Keep simple** - Reliability over cleverness

## Last Updated
2026-02-03
