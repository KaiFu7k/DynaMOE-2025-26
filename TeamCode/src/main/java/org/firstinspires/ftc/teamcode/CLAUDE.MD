# CLAUDE.MD - AI Assistant Guide

This file provides context and guidelines for AI assistants working with the DynaMOE Team 19889 FTC robot codebase.

## Project Context

**Team:** DynaMOE 19889
**Competition:** FIRST Tech Challenge - DECODE (2025-26 season)
**Language:** Java
**Framework:** FTC SDK (FIRST Tech Challenge)
**Robot Type:** Mecanum drive with launcher system

## Code Architecture

This is a **modular subsystem-based architecture**. Each subsystem is independent and can be used in both Autonomous and TeleOp modes.

### Key Architectural Principles
1. **Subsystems are independent** - Each controls one aspect of the robot
2. **RobotHardware aggregates subsystems** - Single point to access all robot functionality
3. **Shared code** - Same subsystems used in Auton and TeleOp
4. **Hardware config names are critical** - Must match Driver Hub configuration

### Directory Map
```
opmodes/          - Competition OpModes (entry points)
robot/            - RobotHardware.java (main aggregator)
subsystems/       - Drivetrain, Launcher, Intake, ArtifactManager
util/             - RobotEnums, FieldPositions, MotifDetector, RobotLogger
pedroPathing/     - Autonomous navigation constants
states/           - AprilTag vision and state machines
```

## Important Files

### Core Files (Modify with Care)
- `robot/RobotHardware.java` - Main robot class, aggregates all subsystems
- `subsystems/Drivetrain.java` - Mecanum drive control
- `subsystems/Launcher.java` - Dual motor launcher with feeders
- `subsystems/Intake.java` - Intake motor control
- `subsystems/ArtifactManager.java` - Artifact tracking and MOTIF detection

### Competition OpModes (Actively Used)
- `opmodes/DynaMOE_19889_Auton.java` - Current autonomous
- `opmodes/DynaMOE_19889_TeleOp.java` - Driver control mode

### Configuration Files
- `util/FieldPositions.java` - Field coordinates
- `pedroPathing/Constants.java` - Pedro Pathing configuration
- `util/RobotEnums.java` - Shared enums

### Legacy/Reference Files (Don't Delete)
- `opmodes/DynaMOE_19889_Auton_Old.java` - Original auton (backup)
- `opmodes/FieldCentricMecanumDrive.java` - Reference drive code
- `opmodes/RobotCentricMecanumDrive.java` - Reference drive code

## Hardware Configuration

**CRITICAL:** These names must exactly match the Driver Hub hardware configuration!

### Motors
- `left_front_drive`, `right_front_drive`, `left_back_drive`, `right_back_drive` (DcMotor)
- `left_launcher`, `right_launcher` (DcMotorEx - requires velocity control)
- `intake` (DcMotor)

### Servos
- `left_feeder`, `right_feeder` (CRServo - continuous rotation)
- `diverter` (Servo - standard servo)

### Sensors
- IMU (built-in to Control Hub)
- AprilTag camera (see states/ folder)

## Common Modifications

### Tuning Launcher Speeds
Edit `subsystems/Launcher.java`:
```java
LAUNCHER_CLOSE_TARGET_VELOCITY = 1200  // RPM for close shots
LAUNCHER_FAR_TARGET_VELOCITY = 1350    // RPM for far shots
```

### Adjusting Field Positions
Edit `util/FieldPositions.java` - coordinates are in inches

### Changing Artifact Preload Configuration
Edit `subsystems/ArtifactManager.java` or call:
```java
robot.artifactManager.configureDefaultPreload();
```

### Modifying Drive Sensitivity
Edit `opmodes/DynaMOE_19889_TeleOp.java`:
```java
DRIVE_SPEED_MULTIPLIER = 0.8
ROTATION_SPEED_MULTIPLIER = 0.6
```

## Development Guidelines

### When Adding New Features
1. Consider if it belongs in a subsystem (robot mechanism) or OpMode (driver control)
2. Use the existing subsystem pattern if adding new hardware
3. Add to RobotHardware if it's a new subsystem
4. Always test independently before integrating

### When Debugging
1. Enable debug logging: `robot.logger.setMinLogLevel(RobotLogger.LogLevel.DEBUG)`
2. Use telemetry: `robot.updateTelemetry()`
3. Check hardware config names match exactly
4. Verify motor directions in subsystem init methods

### Code Style
- Use descriptive variable names
- Follow existing naming conventions (camelCase for methods/variables, PascalCase for classes)
- Add comments for non-obvious logic
- Keep OpModes simple - complex logic goes in subsystems

### Testing Strategy
1. Test individual subsystems first (create test OpModes in opmodes/)
2. Test TeleOp before Autonomous
3. Always have a "stop all" button accessible during testing
4. Keep backup OpModes (never delete old working code during competition season)

## FTC-Specific Knowledge

### OpMode Lifecycle
```
init() -> init_loop() -> [wait for start] -> start() -> loop() -> stop()
```

### Telemetry
```java
telemetry.addData("Key", value);
telemetry.update();
```

### Pedro Pathing
- Autonomous navigation library
- Configured in `pedroPathing/Constants.java`
- Uses field coordinates from `FieldPositions.java`

### MOTIF System
- Game-specific: Pattern detection for artifact scoring
- Managed by ArtifactManager
- Three patterns: GPP, GGP, PGG (G=Green, P=Purple)

## Competition Day Checklist

### Pre-Match
1. Verify hardware config on Driver Hub matches code
2. Test TeleOp controls
3. Test autonomous path selection
4. Check battery levels
5. Verify correct alliance color selected

### During Match
- TeleOp is primary OpMode
- Autonomous uses position + MOTIF selection

### Post-Match
- Review logs if issues occurred
- Note any needed tuning adjustments

## Common Issues

### Motors Not Moving
- Check hardware config names
- Verify motor directions (set in subsystem init)
- Check power is reaching motors (use telemetry)

### Launcher Not Reaching Speed
- Check battery voltage (low battery = low velocity)
- Verify DcMotorEx vs DcMotor in config
- Tune PID in Launcher.java

### Drive Acting Strange
- Check IMU calibration (field-centric mode)
- Verify motor directions
- Check for stick drift (controller issue)

### Autonomous Path Wrong
- Verify FieldPositions coordinates
- Check starting position matches selected option
- Review Pedro Pathing constants

## Working with This Codebase

### Safe Modifications
- Tuning constants (velocities, positions, multipliers)
- Adding telemetry
- Creating new test OpModes
- Adjusting controller mappings

### Risky Modifications (Test Thoroughly)
- Changing subsystem architecture
- Modifying hardware config names
- Changing OpMode lifecycle methods
- Altering Pedro Pathing configuration

### Don't Modify During Competition
- Core subsystem logic (Drivetrain, Launcher, etc.)
- RobotHardware initialization
- Tested autonomous paths
- Working TeleOp controls

## Resources

- **FTC SDK Documentation:** https://github.com/FIRST-Tech-Challenge/FtcRobotController
- **Game Manual:** Official FTC DECODE game rules
- **Pedro Pathing:** https://github.com/pedropathing/pedroPathing
- **Team Repository:** (add your repo URL here)

## Notes for AI Assistants

1. **Preserve competition-ready code** - Never modify working OpModes without explicit permission
2. **Test OpModes first** - Suggest creating test OpModes for new features
3. **Hardware safety** - Never suggest code that could damage hardware (excessive speeds, conflicting commands)
4. **FTC rules compliance** - Code must follow FTC software rules
5. **Keep it simple** - Competition code should be reliable, not clever
6. **Backup strategy** - Suggest keeping old working code before major refactors

## Last Updated
2026-01-15
