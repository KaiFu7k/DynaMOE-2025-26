package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;

/**
 * TeleOp OpMode for DynaMOE Team 19889
 * DECODE Game - Driver-controlled period
 *
 * Uses the same subsystem architecture as autonomous for consistency!
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
 *
 * === GAMEPAD 2 (Operator) - Optional ===
 * (Can add operator controls here if needed)
 */
@TeleOp(name = "DynaMOE 19889 TeleOp", group = "TeleOp")
public class DynaMOE_19889_TeleOp extends LinearOpMode {

    // Hardware and subsystems
    private RobotHardware robot;
    private Follower follower;

    // Drive control
    private boolean fieldCentric = true;  // Toggle with dpad_left/dpad_right

    // Launcher state
    private boolean launcherActive = false;

    @Override
    public void runOpMode() {
        // Initialize robot hardware
        robot = new RobotHardware(telemetry);
        robot.init(hardwareMap);

        // Initialize Pedro Pathing (for field-centric drive)
        follower = Constants.createFollower(hardwareMap);

        robot.logger.info("TeleOp", "Initialization complete");

        telemetry.addLine("=== DynaMOE 19889 TELEOP ===");
        telemetry.addLine("Robot initialized and ready!");
        telemetry.addLine();
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        robot.logger.info("TeleOp", "TeleOp started");

        // Main control loop
        while (opModeIsActive()) {
            // Update follower for field-centric drive
            follower.update();

            // Update subsystems
            robot.updateSubsystems();

            // Handle driver controls
            handleDriveControls();
            handleIntakeControls();
            handleLauncherControls();

            // Update telemetry
            updateTelemetry();

            // Small delay to prevent CPU hogging
            sleep(10);
        }

        // Clean up on stop
        robot.stopAllSubsystems();
        robot.logger.info("TeleOp", "Stopped");
    }

    // ==================== DRIVE CONTROLS ====================

    private void handleDriveControls() {
        // Get gamepad inputs
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;  // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Toggle field-centric mode
        if (gamepad1.dpad_left) {
            fieldCentric = false;
            sleep(200);  // Debounce
        } else if (gamepad1.dpad_right) {
            fieldCentric = true;
            sleep(200);
        }

        // Calculate motor powers
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

        if (fieldCentric) {
            // Field-centric drive (uses Pedro Pathing heading)
            double botHeading = follower.getHeading();

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontPower = (rotY + rotX + rx) / denominator;
            leftBackPower = (rotY - rotX + rx) / denominator;
            rightFrontPower = (rotY - rotX - rx) / denominator;
            rightBackPower = (rotY + rotX - rx) / denominator;
        } else {
            // Robot-centric drive
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFrontPower = (y + x + rx) / denominator;
            leftBackPower = (y - x + rx) / denominator;
            rightFrontPower = (y - x - rx) / denominator;
            rightBackPower = (y + x - rx) / denominator;
        }

        // Apply powers to drivetrain
        robot.drivetrain.setPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    // ==================== INTAKE CONTROLS ====================

    private void handleIntakeControls() {
        if (gamepad1.right_trigger > 0.5) {
            // Intake
            robot.intake.intake();
        } else if (gamepad1.left_trigger > 0.5) {
            // Outtake
            robot.intake.outtake();
        } else {
            // Stop
            robot.intake.stop();
        }
    }

    // ==================== LAUNCHER CONTROLS ====================

    private void handleLauncherControls() {
        // A button: Spin up launchers (close shot)
        if (gamepad1.a && !launcherActive) {
            robot.launcher.spinUp(true);  // Close shot
            launcherActive = true;
            robot.logger.info("Launcher", "Spinning up (close shot)");
        }

        // B button: Stop launchers
        if (gamepad1.b && launcherActive) {
            robot.launcher.stop();
            launcherActive = false;
            robot.logger.info("Launcher", "Stopped");
        }

        // X button: Feed left launcher
        if (gamepad1.x && launcherActive && robot.launcher.isReady()) {
            robot.launcher.startFeed(LauncherSide.LEFT);
            robot.logger.info("Launcher", "Feeding LEFT");
        }

        // Y button: Feed right launcher
        if (gamepad1.y && launcherActive && robot.launcher.isReady()) {
            robot.launcher.startFeed(LauncherSide.RIGHT);
            robot.logger.info("Launcher", "Feeding RIGHT");
        }

        // Optional: Gamepad 2 controls for operator
        // Dpad up/down: Adjust launcher velocity
        if (gamepad2.dpad_up) {
            robot.launcher.spinUp(false);  // Far shot
            robot.logger.info("Launcher", "Switched to FAR shot");
            sleep(200);
        } else if (gamepad2.dpad_down) {
            robot.launcher.spinUp(true);  // Close shot
            robot.logger.info("Launcher", "Switched to CLOSE shot");
            sleep(200);
        }
    }

    // ==================== TELEMETRY ====================

    private void updateTelemetry() {
        telemetry.addLine("=== DYNAMOE 19889 TELEOP ===");
        telemetry.addLine();

        // Drive mode
        telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getHeading()));
        telemetry.addLine();

        // Launcher status
        telemetry.addData("Launcher", launcherActive ? "ACTIVE" : "Stopped");
        if (launcherActive) {
            double[] velocities = robot.launcher.getVelocities();
            telemetry.addData("  Left", "%.0f RPM", velocities[0]);
            telemetry.addData("  Right", "%.0f RPM", velocities[1]);
            telemetry.addData("  Ready", robot.launcher.isReady() ? "YES" : "NO");
        }
        telemetry.addLine();

        // Intake status
        telemetry.addData("Intake", robot.intake.isRunning() ? "RUNNING" : "Stopped");
        telemetry.addData("  Power", "%.2f", robot.intake.getCurrentPower());
        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("Left Stick: Drive");
        telemetry.addLine("Right Stick: Rotate");
        telemetry.addLine("RT: Intake | LT: Outtake");
        telemetry.addLine("A: Launchers ON | B: OFF");
        telemetry.addLine("X: Feed Left | Y: Feed Right");
        telemetry.addLine("D-Pad L/R: Toggle Drive Mode");

        telemetry.update();
    }
}
