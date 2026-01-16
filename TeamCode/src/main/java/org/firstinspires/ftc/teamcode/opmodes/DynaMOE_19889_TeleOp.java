package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;

/**
 * TeleOp OpMode for DynaMOE Team 19889
 * DECODE Game - Driver-controlled period (2-minute match phase after autonomous)
 *
 * ARCHITECTURE:
 * - Uses the same subsystem architecture as autonomous for code consistency
 * - All robot mechanisms controlled through RobotHardware aggregator
 * - Field-centric drive uses Pedro Pathing for IMU heading tracking
 * - Supports optional 2-gamepad setup (driver + operator)
 *
 * CONTROLS:
 * === GAMEPAD 1 (Driver) ===
 * Left Stick: Forward/backward and strafe
 * Right Stick: Rotation
 * Right Trigger: Intake artifacts
 * Left Trigger: Outtake artifacts
 * A: Spin up launchers (at current speed)
 * B: Stop launchers
 * X: Feed left launcher
 * Y: Feed right launcher
 * D-Pad Up/Down: Increase/decrease launcher speed (50 RPM increments)
 * D-Pad Left/Right: Toggle robot/field-centric drive
 *
 * === GAMEPAD 2 (Operator) - Optional ===
 * D-Pad Up: Preset far shot (1350 RPM)
 * D-Pad Down: Preset close shot (1200 RPM)
 * D-Pad Left: Preset max range (1550 RPM)
 * D-Pad Right: Preset min safe (900 RPM)
 *
 * MATCH WORKFLOW:
 * 1. Autonomous runs first (30 seconds)
 * 2. TeleOp begins automatically (2 minutes)
 * 3. Drivers control robot to score artifacts and complete objectives
 * 4. OpMode stops automatically at match end
 */
@TeleOp(name = "DynaMOE 19889 TeleOp", group = "TeleOp")
public class DynaMOE_19889_TeleOp extends LinearOpMode {

    // Hardware and subsystems
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor rightFront = null;
    private DcMotor intakeMotor = null;
    private RobotHardware robot;
    private Follower follower;

    // Drive control
    private boolean fieldCentric = false;  // Toggle with dpad_left/dpad_right

    // Launcher state
    private boolean launcherActive = false;

    // Manual launcher speed control (Plan A)
    private static final double LAUNCHER_MIN_SPEED = 800;   // Minimum safe RPM
    private static final double LAUNCHER_MAX_SPEED = 1600;  // Maximum safe RPM
    private static final double LAUNCHER_SPEED_INCREMENT = 50;  // RPM change per button press
    private double manualLauncherSpeed = 1200;  // Starting speed (close shot default)

    // Motor power tracking for telemetry
    private double currentLFPower = 0;
    private double currentRFPower = 0;
    private double currentLBPower = 0;
    private double currentRBPower = 0;

    /**
     * Main OpMode execution method - called by FTC SDK when OpMode is selected.
     *
     * OPMODE LIFECYCLE:
     * 1. init() - Setup hardware, calibrate sensors
     * 2. init_loop() - Waiting for START button (not used here)
     * 3. start() - Match begins (triggered by waitForStart())
     * 4. loop() - Main control loop runs continuously
     * 5. stop() - Match ends or STOP pressed
     *
     * INITIALIZATION PHASE:
     * - Create RobotHardware instance (aggregates all subsystems)
     * - Initialize all motors, servos, sensors via hardwareMap
     * - Setup Pedro Pathing for IMU-based field-centric drive
     * - Display ready message on Driver Hub
     *
     * LOOP PHASE:
     * - Runs at ~100Hz (every 10ms)
     * - Updates sensors and subsystem states
     * - Processes gamepad inputs
     * - Sends commands to hardware
     * - Updates driver display
     *
     * CLEANUP PHASE:
     * - Stop all motors and servos
     * - Log final status
     */
    @Override
    public void runOpMode() {
        // === INITIALIZATION PHASE ===

        // Initialize Pedro Pathing follower for field-centric drive
        // Provides IMU heading and pose tracking
        follower = Constants.createFollower(hardwareMap);

        // Initialize robot hardware (all subsystems: drivetrain, launcher, intake, etc.)
        robot = new RobotHardware(telemetry);
        robot.init(hardwareMap);

        robot.logger.info("TeleOp", "Initialization complete");

        // Display ready status to drivers
        telemetry.addLine("=== DynaMOE 19889 TELEOP ===");
        telemetry.addLine("Robot initialized and ready!");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        // Wait for driver to press START button on Driver Hub
        // During this time, drivers can review telemetry and prepare
        waitForStart();

        // Safety check: Exit if STOP was pressed during init
        if (isStopRequested()) return;

        robot.logger.info("TeleOp", "TeleOp started");

        // === MAIN CONTROL LOOP ===
        // Runs continuously until match ends or STOP is pressed
        while (opModeIsActive()) {
            // Update Pedro Pathing pose tracking (for field-centric drive)
            // HYBRID APPROACH: We use Pedro ONLY for IMU heading/position tracking
            // Pedro does NOT control motors - we handle that via robot.drivetrain
            // This avoids conflicts with brake mode and gives us full motor control
            follower.update();

            // Update all subsystems (refresh sensor readings, update state machines)
            robot.updateSubsystems();

            // Process gamepad inputs and send commands to hardware
            handleDriveControls();      // Mecanum drive (field/robot-centric)
            handleIntakeControls();     // Artifact intake/outtake
            handleLauncherControls();   // Launcher spin-up and feeding

            // Update Driver Hub display with current status
            updateTelemetry();

            // Small delay to prevent CPU overload
            // Loop runs at ~100Hz (every 10ms)
            sleep(10);
        }

        // === CLEANUP PHASE ===
        // Match ended or STOP pressed - safely shut down all systems
        robot.stopAllSubsystems();
        robot.logger.info("TeleOp", "Stopped");
    }

    // ==================== DRIVE CONTROLS ====================

    /**
     * Handles mecanum drive controls with both field-centric and robot-centric modes.
     *
     * MECANUM DRIVE BASICS:
     * - 4 wheels at 45° angles allow omnidirectional movement
     * - Left stick Y/X controls translation (forward/backward and left/right strafe)
     * - Right stick X controls rotation
     * - All movements can happen simultaneously
     *
     * ROBOT-CENTRIC MODE:
     * - Forward on stick = robot moves forward (relative to robot's front)
     * - If robot is facing sideways, "forward" is still robot's perspective
     *
     * FIELD-CENTRIC MODE:
     * - Forward on stick = robot moves toward far side of field (fixed reference)
     * - Robot's heading is compensated using IMU, so controls stay consistent
     * - Driver doesn't need to adjust for robot rotation
     */
    private void handleDriveControls() {
        // Get gamepad inputs
        // Y axis: Forward/backward (negated because gamepad Y is inverted)
        double y = -gamepad1.left_stick_y;

        // X axis: Strafe left/right
        // Multiplied by 1.1 to counteract imperfect strafing (mecanum wheels drift slightly)
        double x = gamepad1.left_stick_x * 1.1;

        // Rotation: Right stick X controls spinning in place
        double rx = gamepad1.right_stick_x;

        // Toggle between field-centric and robot-centric drive modes
        if (gamepad1.dpad_left) {
            fieldCentric = false;
            sleep(200);  // Debounce to prevent multiple toggles
        } else if (gamepad1.dpad_right) {
            fieldCentric = true;
            sleep(200);
        }

        // Calculate motor powers based on drive mode
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

        if (fieldCentric) {
            // === FIELD-CENTRIC DRIVE ===
            // Compensate for robot's heading so driver's perspective stays fixed to field

            // Get current robot heading from IMU (via Pedro Pathing)
            // 0° = initial orientation, increases counterclockwise
            double botHeading = follower.getHeading();

            // Rotate the joystick input vector by the robot's heading
            // This transforms field-relative controls to robot-relative controls
            // Uses 2D rotation matrix: [cos(θ) -sin(θ)] [x]
            //                          [sin(θ)  cos(θ)] [y]
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            // Calculate wheel powers using mecanum drive kinematics
            // Each wheel contributes to: forward/backward + strafe + rotation
            // Denominator normalizes so max power = 1.0 (prevents motor saturation)
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontPower = (rotY + rotX + rx) / denominator;   // Forward + Strafe Right + Rotate CCW
            leftBackPower = (rotY - rotX + rx) / denominator;    // Forward + Strafe Left + Rotate CCW
            rightFrontPower = (rotY - rotX - rx) / denominator;  // Forward + Strafe Left + Rotate CW
            rightBackPower = (rotY + rotX - rx) / denominator;   // Forward + Strafe Right + Rotate CW
        } else {
            // === ROBOT-CENTRIC DRIVE ===
            // Use raw joystick inputs without heading compensation
            // Driver must mentally adjust for robot's orientation

            // Same mecanum kinematics, but using unrotated x/y inputs
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFrontPower = (y + x + rx) / denominator;
            leftBackPower = (y - x + rx) / denominator;
            rightFrontPower = (y - x - rx) / denominator;
            rightBackPower = (y + x - rx) / denominator;
        }

        // Store power values for telemetry display
        currentLFPower = leftFrontPower;
        currentRFPower = rightFrontPower;
        currentLBPower = leftBackPower;
        currentRBPower = rightBackPower;

        // Send calculated powers to drivetrain subsystem
        robot.drivetrain.setPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    // ==================== INTAKE CONTROLS ====================


    private void handleIntakeControls() {
        if (gamepad1.right_trigger > 0.5) {
            // Intake mode: Pull artifacts into robot
            robot.intake.intake();
        } else if (gamepad1.left_trigger > 0.5) {
            // Outtake mode: Eject artifacts from robot
            // Useful for clearing jams or removing wrong-colored artifacts
            robot.intake.outtake();
        } else {
            // Stop mode: No trigger pressed, motor off
            robot.intake.stop();
        }
    }
    // ==================== LAUNCHER CONTROLS ====================

    /**
     * Handles dual launcher system controls for scoring artifacts.
     *
     * LAUNCHER SYSTEM:
     * - Two flywheels (left and right) spin at high velocity to launch artifacts
     * - Feeders push artifacts from storage into the spinning flywheels
     * - System must reach target velocity before feeding (prevents weak launches)
     * - MANUAL SPEED CONTROL: Driver adjusts RPM in real-time (Plan A)
     *
     * MANUAL SPEED CONTROL (PLAN A):
     * - D-Pad Up/Down: Adjust speed in 50 RPM increments (800-1600 RPM range)
     * - Speed can be adjusted before spin-up OR while launchers are running
     * - Current speed shown in telemetry at all times
     * - Gamepad 2 has quick presets for common distances (optional)
     *
     * WORKFLOW:
     * 1. Adjust speed with D-Pad Up/Down until target RPM is shown
     * 2. Press A to spin up launchers at current speed
     * 3. Wait for "Ready" indicator (motors reached target velocity)
     * 4. Press X or Y to feed left/right launcher
     * 5. Adjust speed mid-match if needed (D-Pad Up/Down while spinning)
     * 6. Press B to stop launchers when done (saves battery)
     *
     * SAFETY FEATURES:
     * - Speed clamped to safe range (800-1600 RPM)
     * - Can't feed until launchers are at speed (prevents jams)
     * - State tracking prevents accidental multiple spin-ups
     * - Separate left/right feeding for artifact management
     */
    private void handleLauncherControls() {
        // === MANUAL SPEED ADJUSTMENT (PLAN A) ===
        // D-Pad Up/Down: Increase/decrease launcher speed in 50 RPM increments
        // Works whether launchers are spinning or not (pre-set before spin-up)
        if (gamepad1.dpad_up) {
            manualLauncherSpeed += LAUNCHER_SPEED_INCREMENT;
            // Clamp to maximum safe speed
            if (manualLauncherSpeed > LAUNCHER_MAX_SPEED) {
                manualLauncherSpeed = LAUNCHER_MAX_SPEED;
            }

            // If launchers are already spinning, apply new speed immediately
            if (launcherActive) {
                robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
                robot.logger.info("Launcher", String.format("Speed increased to %.0f RPM", manualLauncherSpeed));
            }
            sleep(150);  // Debounce
        }
        else if (gamepad1.dpad_down) {
            manualLauncherSpeed -= LAUNCHER_SPEED_INCREMENT;
            // Clamp to minimum safe speed
            if (manualLauncherSpeed < LAUNCHER_MIN_SPEED) {
                manualLauncherSpeed = LAUNCHER_MIN_SPEED;
            }

            // If launchers are already spinning, apply new speed immediately
            if (launcherActive) {
                robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
                robot.logger.info("Launcher", String.format("Speed decreased to %.0f RPM", manualLauncherSpeed));
            }
            sleep(150);  // Debounce
        }

        // A button: Spin up launchers at current manual speed
        // Uses manualLauncherSpeed instead of preset close/far shot
        if (gamepad1.a && !launcherActive) {
            robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            launcherActive = true;
            robot.logger.info("Launcher", String.format("Spinning up at %.0f RPM", manualLauncherSpeed));
        }

        // B button: Stop launchers
        // Stops flywheels and feeders, saves battery power
        // Manual speed setting is preserved for next spin-up
        if (gamepad1.b && launcherActive) {
            robot.launcher.stop();
            launcherActive = false;
            robot.logger.info("Launcher", "Stopped");
        }

        // X button: Feed left launcher
        // Three conditions must be met:
        // 1. X is pressed
        // 2. Launchers are active (spinning)
        // 3. Launchers are ready (at target velocity)
        if (gamepad1.x && launcherActive && robot.launcher.isReady()) {
            robot.launcher.startFeed(LauncherSide.LEFT);
            robot.logger.info("Launcher", "Feeding LEFT");
        }

        // Y button: Feed right launcher
        // Same conditions as X button, but feeds right side
        // Allows strategic artifact management (e.g., launch specific colors)
        if (gamepad1.y && launcherActive && robot.launcher.isReady()) {
            robot.launcher.startFeed(LauncherSide.RIGHT);
            robot.logger.info("Launcher", "Feeding RIGHT");
        }

        // === GAMEPAD 2: OPERATOR CONTROLS (OPTIONAL) ===
        // Backup controls for 2-driver setup
        // Also allows larger speed jumps with presets

        // D-pad up: Quick preset - Far shot (1350 RPM)
        if (gamepad2.dpad_up) {
            manualLauncherSpeed = 1350;
            if (launcherActive) {
                robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            }
            robot.logger.info("Launcher", "Preset: FAR shot (1350 RPM)");
            sleep(200);  // Debounce
        }

        // D-pad down: Quick preset - Close shot (1200 RPM)
        else if (gamepad2.dpad_down) {
            manualLauncherSpeed = 1200;
            if (launcherActive) {
                robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            }
            robot.logger.info("Launcher", "Preset: CLOSE shot (1200 RPM)");
            sleep(200);  // Debounce
        }

        // D-pad left: Quick preset - Max range (1550 RPM)
        else if (gamepad2.dpad_left) {
            manualLauncherSpeed = 1550;
            if (launcherActive) {
                robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            }
            robot.logger.info("Launcher", "Preset: MAX range (1550 RPM)");
            sleep(200);  // Debounce
        }

        // D-pad right: Quick preset - Min safe (900 RPM)
        else if (gamepad2.dpad_right) {
            manualLauncherSpeed = 900;
            if (launcherActive) {
                robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            }
            robot.logger.info("Launcher", "Preset: MIN safe (900 RPM)");
            sleep(200);  // Debounce
        }
    }

    // ==================== TELEMETRY ====================

    /**
     * Updates the Driver Hub display with real-time robot status and controls.
     *
     * TELEMETRY PURPOSE:
     * - Provides drivers with essential feedback during match
     * - Shows system status (drive mode, launcher speed, intake state)
     * - Displays control reminders for quick reference
     * - Updates every loop cycle (~100Hz)
     *
     * DISPLAY SECTIONS:
     * 1. Drive Status: Current mode (field/robot-centric) and robot heading
     * 2. Launcher Status: Active state, wheel velocities, ready indicator
     * 3. Intake Status: Running state and current motor power
     * 4. Controls Reference: Button mapping reminder
     *
     * CRITICAL INFO FOR DRIVERS:
     * - "Ready: YES" means launchers are at speed, safe to feed
     * - Heading shows robot orientation (0° = starting direction)
     * - Velocities show both launchers (should be close to target RPM)
     */
    private void updateTelemetry() {
        // Header
        telemetry.addLine("=== DYNAMOE 19889 TELEOP ===");
        telemetry.addLine();

        // === DRIVE STATUS ===
        // Shows which control mode is active and current robot orientation
        telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getHeading()));
        telemetry.addLine();

        // === MOTOR POWERS (DEBUG) ===
        // Display power being sent to each wheel (helps diagnose drive issues)
        telemetry.addLine("--- MOTOR POWERS ---");
        telemetry.addData("LF (Left Front)", "%.2f", currentLFPower);
        telemetry.addData("RF (Right Front)", "%.2f", currentRFPower);
        telemetry.addData("LB (Left Back)", "%.2f", currentLBPower);
        telemetry.addData("RB (Right Back)", "%.2f", currentRBPower);
        telemetry.addLine();

        // === BRAKE MODE DIAGNOSTIC ===
        // Check if Pedro Pathing is overriding our brake mode settings
        telemetry.addLine("--- BRAKE MODE STATUS ---");
        telemetry.addData("LF Brake Mode", robot.drivetrain.getLeftFrontDrive().getZeroPowerBehavior());
        telemetry.addData("RF Brake Mode", robot.drivetrain.getRightFrontDrive().getZeroPowerBehavior());
        telemetry.addData("LB Brake Mode", robot.drivetrain.getLeftBackDrive().getZeroPowerBehavior());
        telemetry.addData("RB Brake Mode", robot.drivetrain.getRightBackDrive().getZeroPowerBehavior());
        telemetry.addLine();

        // === LAUNCHER STATUS ===
        // Most critical info: tells drivers if system is ready to score
        telemetry.addData("Launcher", launcherActive ? "ACTIVE" : "Stopped");

        // Always show target speed (even when stopped, so driver knows what's set)
        telemetry.addData("Target Speed", "%.0f RPM", manualLauncherSpeed);

        if (launcherActive) {
            // Show current velocities for both flywheels
            double[] velocities = robot.launcher.getVelocities();
            telemetry.addData("  Left", "%.0f RPM", velocities[0]);
            telemetry.addData("  Right", "%.0f RPM", velocities[1]);

            // "Ready" indicator: Green light to feed artifacts
            // Only shows YES when both motors are within tolerance of target
            telemetry.addData("  Ready", robot.launcher.isReady() ? "YES" : "NO");
        }
        telemetry.addLine();

        // === INTAKE STATUS ===
        // Shows if intake is actively pulling/pushing artifacts
        telemetry.addData("Intake", robot.intake.isRunning() ? "RUNNING" : "Stopped");
        telemetry.addData("  Power", "%.2f", robot.intake.getCurrentPower());
        telemetry.addLine();

        // === CONTROLS REMINDER ===
        // Quick reference for drivers (especially useful for backups/new drivers)
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Left Stick: Drive");
        telemetry.addLine("Right Stick: Rotate");
        telemetry.addLine("RT: Intake | LT: Outtake");
        telemetry.addLine("A: Launchers ON | B: OFF");
        telemetry.addLine("X: Feed Left | Y: Feed Right");
        telemetry.addLine("D-Pad U/D: Speed +/- 50 RPM");
        telemetry.addLine("D-Pad L/R: Toggle Drive Mode");

        // Push all telemetry data to Driver Hub screen
        telemetry.update();
    }
}
