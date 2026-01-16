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
 * A: Spin up launchers (close shot)
 * B: Stop launchers
 * X: Feed left launcher
 * Y: Feed right launcher
 * D-Pad Left/Right: Toggle robot/field-centric drive
 *
 * === GAMEPAD 2 (Operator) - Optional ===
 * D-Pad Up: Switch to far shot
 * D-Pad Down: Switch to close shot
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
            // Must be called every loop to maintain accurate heading
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
     * - Two velocity presets: CLOSE shot and FAR shot (different RPMs)
     *
     * WORKFLOW:
     *
     * 1. Driver presses A to spin up launchers
     * 2. Wait for "Ready" indicator (motors reached target velocity)
     * 3. Press X or Y to feed left/right launcher
     * 4. Press B to stop launchers when done (saves battery)
     *
     * SAFETY FEATURES:
     * - Can't feed until launchers are at speed (prevents jams)
     * - State tracking prevents accidental multiple spin-ups
     * - Separate left/right feeding for artifact management
     */
    private void handleLauncherControls() {
        // A button: Spin up launchers (close shot)
        // Only activates if launchers aren't already running (prevents re-initialization)
        if (gamepad1.a && !launcherActive) {
            robot.launcher.spinUp(true);  // true = close shot velocity
            launcherActive = true;
            robot.logger.info("Launcher", "Spinning up (close shot)");
        }

        // B button: Stop launchers
        // Stops flywheels and feeders, saves battery power
        // Only triggers if launchers are currently active
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

        // === GAMEPAD 2: OPERATOR CONTROLS ===
        // Optional second driver can adjust shot distance during match
        // Useful for adapting to different scoring positions

        // D-pad up: Switch to far shot (higher velocity)
        if (gamepad2.dpad_up) {
            robot.launcher.spinUp(false);  // false = far shot velocity
            robot.logger.info("Launcher", "Switched to FAR shot");
            sleep(200);  // Debounce
        }

        // D-pad down: Switch to close shot (lower velocity)
        else if (gamepad2.dpad_down) {
            robot.launcher.spinUp(true);  // true = close shot velocity
            robot.logger.info("Launcher", "Switched to CLOSE shot");
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

        // === LAUNCHER STATUS ===
        // Most critical info: tells drivers if system is ready to score
        telemetry.addData("Launcher", launcherActive ? "ACTIVE" : "Stopped");
        if (launcherActive) {
            // Show current velocities for both flywheels
            // Drivers should verify these match target (1200 or 1350 RPM)
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
        telemetry.addLine("D-Pad L/R: Toggle Drive Mode");

        // Push all telemetry data to Driver Hub screen
        telemetry.update();
    }
}
