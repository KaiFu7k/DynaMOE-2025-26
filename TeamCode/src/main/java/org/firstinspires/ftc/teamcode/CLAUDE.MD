# CLAUDE.MD - AI Assistant Guide

Context for AI assistants working with DynaMOE Team 19889 FTC robot code.

## Project Info
- **Team:** DynaMOE 19889 | **Season:** DECODE 2025-26
- **Language:** Java | **Framework:** FTC SDK + Ivy (Pedro Pathing command framework)
- **Robot:** Mecanum drive with dual launcher

## Architecture

Modular subsystem-based. Subsystems are independent and shared between Auton/TeleOp.

```
opmodes/       - OpModes (DynaMOE_19889_Auton, DynaMOE_19889_TeleOp, LauncherVelocityTuner)
robot/         - RobotHardware.java (aggregates subsystems)
subsystems/    - Drivetrain, Launcher, Intake, ArtifactManager, LauncherAssist
util/          - RobotEnums, FieldPositions, MotifDetector, RobotLogger, RobotState
pedroPathing/  - Autonomous navigation (Pedro Pathing follower + constants)
states/        - AprilTag vision
```

### Ivy Command Framework (Autonomous)

Autonomous uses the **Ivy** command library (`com.pedropathing:ivy`) — a command-based scheduler on top of Pedro Pathing. Key concepts:

- **Scheduler**: `Scheduler.reset()` in init, `Scheduler.execute()` in main loop. Commands are registered via `schedule()`.
- **Commands**: `instant()` (run once), `waitMs()`, `waitUntil()`, `infinite()` (runs every tick)
- **Groups**: `sequential()`, `parallel()` (all must finish), `race()` (first to finish wins)
- **Pedro integration**: `follow(follower, pathChain)` wraps path following as a command
- **Background commands**: `infinite(() -> follower.update())` and `infinite(() -> robot.updateSubsystems())` run every tick alongside the main sequence

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
| Perimeter launch | `DynaMOE_19889_Auton.java` | Auto-calculated from LERP table |
| Goal-side final launch | `DynaMOE_19889_Auton.java` | Auto-calculated from LERP table |
| Spinup delay | `DynaMOE_19889_Auton.java` | 800 ms |
| Launcher spinup timeout | `DynaMOE_19889_Auton.java` | 1500 ms |
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

## Autonomous Routines

### Goal-Side (12 artifacts)
1. Score 3 preloads at (48,96)/(96,96) — close shot 1150 RPM
2. Top spike (Y=84) — intake at x=18, return to launch, score
3. Middle spike (Y=58) — intake at x=15, back up to x=33 (obstruction at 0-12,72), return, score
4. Bottom spike (Y=36) — intake at x=15, go to final launch (60,108)/(84,108) at 1170 RPM
5. Park at final launch position (off launch zone lines)

### Perimeter-Side (Stub — preloads only)
1. Wait 24 seconds
2. Score 3 preloads at (48,15)/(96,15) — LERP table velocity
3. Park at (85,35)

### Key Auton Features
- **Ivy command-based**: Entire auto is a `schedule(sequential(...))` with composable commands
- **Pre-built paths**: All PathChains built during init from fixed poses (not dynamic)
- **Parallel spinup**: `parallel(follow(...), sequential(waitMs(delay), spinUpLauncher()))` spins up launcher while driving
- **Background updates**: `infinite(() -> follower.update())`, `infinite(() -> robot.updateSubsystems())`, and `infinite(() -> RobotState.saveAutonEndPose(...))` run every scheduler tick
- **Launcher readiness**: `race(waitUntil(isReady), waitMs(timeout))` waits for launcher with timeout
- **shootSide command**: `sequential(instant(startFeed), waitMs(900), instant(stopFeeders))` for each feed
- BezierLine paths only (BezierCurve abandoned — unreliable)
- Pose continuously saved for TeleOp via `RobotState`

## TeleOp Controls

### Single-Button Launch Mode
| Button | Action |
|--------|--------|
| **Right Bumper** | Start launch sequence (auto-align + auto-velocity + intake + fire) |
| **B** | Abort launch sequence |

During launch sequence:
- Drive is locked out (no joystick input)
- Intake runs automatically to seat artifacts
- Robot auto-rotates to face goal via PD controller
- Launcher velocity auto-calculated from distance

### Drive & Intake
| Button | Action |
|--------|--------|
| Left Stick | Drive |
| Right Stick | Rotate (locked during launch sequence) |
| Right Trigger | Intake |
| Left Trigger | Outtake |
| D-pad L/R | Toggle robot/field centric |

### Manual Launcher (Backup)
| Button | Action |
|--------|--------|
| A | Manual spin up |
| X | Manual feed LEFT |
| Y | Manual feed RIGHT |
| Gamepad2 D-pad U/D | Adjust manual speed +/-50 RPM |

### TeleOp Technical Notes
- `follower.update()` called every loop for odometry; motor powers overwritten by drive controls
- LauncherAssist updated via `robot.updateSubsystems()` (not duplicated)
- During launch sequence: follower still updates pose but drive output is zero/auto-align only

## Auton -> TeleOp Pose Transfer

Robot pose is preserved between Auton and TeleOp via `RobotState.java`:
- Auton saves final pose: `RobotState.saveAutonEndPose(pose, alliance)`
- TeleOp reads it: `RobotState.getAutonEndPose()`
- Pose expires after 5 minutes
- If no valid pose, TeleOp falls back to manual selection

## Field Coordinates

Origin (0,0) at bottom-left. X: 0-144 (left-right), Y: 0-144 (bottom-top).
Heading: 0=right, 90=up, 180=left, 270=down.

| Position | X | Y | Heading |
|----------|---|---|---------|
| Blue Goal Start | 36 | 132 | 90 |
| Red Goal Start | 108 | 132 | 90 |
| Blue Perimeter Start | 48 | 9 | 90 |
| Red Perimeter Start | 96 | 9 | 90 |
| Blue Goal-Side Launch | 48 | 96 | 135 |
| Red Goal-Side Launch | 96 | 96 | 45 |
| Blue Perimeter Launch | 48 | 15 | 110 |
| Red Perimeter Launch | 96 | 15 | 70 |
| Blue Final Launch | 60 | 108 | 149 |
| Red Final Launch | 84 | 108 | 31 |
| Blue Park (Perimeter) | 36 | 12 | 180 |
| Red Park (Perimeter) | 108 | 12 | 0 |
| Blue Goal | 0 | 144 | — |
| Red Goal | 144 | 144 | — |
| Obstruction (Blue) | 0-12 | 72 | — |

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
| Robot spins during TeleOp launch | Check starting pose matches reality, verify no duplicate launcherAssist.update() |
| Robot jiggle during scoring | Ivy handles this — follower.update() runs via infinite command but follow() commands are sequential |
| Spike overshoot into wall | Use separate follow() commands (not chained paths) |
| Scheduler not running | Ensure Scheduler.reset() in init, Scheduler.execute() in while loop |
| Commands not executing | Check schedule() was called after waitForStart() |

## AI Guidelines

1. **Preserve working code** - Don't modify competition OpModes without permission
2. **Test first** - Create test OpModes for new features
3. **Hardware safety** - No excessive speeds or conflicting commands
4. **Keep simple** - Reliability over cleverness
5. **BezierLine only** - BezierCurve was tried and abandoned (unpredictable)
6. **Use Ivy commands** - Autonomous uses Ivy scheduler; don't mix blocking loops with Ivy commands
7. **Pre-build paths** - All paths built during init from fixed poses, not dynamically from current pose

## Last Updated
2026-02-14
