package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;
import org.firstinspires.ftc.teamcode.util.FieldPositions;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * TeleOp OpMode for DynaMOE Team 19889
 *
 * SINGLE-BUTTON LAUNCH MODE:
 * - Right Bumper: Start launch sequence (auto-align + auto-velocity + fire)
 * - B: Abort launch sequence
 *
 * Launch sequence:
 * 1. ALIGNING: Robot auto-rotates to face goal, launcher spins up to calculated velocity
 * 2. FIRING: When aligned and ready, feeds LEFT then RIGHT with slight delay
 * 3. Returns to IDLE automatically
 */
@TeleOp(name = "DynaMOE 19889 TeleOp", group = "TeleOp")
public class DynaMOE_19889_TeleOp extends LinearOpMode {

    // ==================== HARDWARE ====================
    private RobotHardware robot;
    private Follower follower;

    // ==================== LAUNCH STATE MACHINE ====================
    private enum LaunchState {
        IDLE,       // Normal driving, launcher off
        ALIGNING,   // Auto-align active, spinning up launcher
        FIRING_LEFT,  // Feeding left side
        FIRING_RIGHT  // Feeding right side
    }
    private LaunchState launchState = LaunchState.IDLE;
    private ElapsedTime launchTimer = new ElapsedTime();
    private static final double FEED_DELAY_SECONDS = 0.3;  // Delay between L/R feeds
    private static final double LAUNCH_TIMEOUT_SECONDS = 5.0;  // Max time in ALIGNING state

    // ==================== DRIVE STATE ====================
    private boolean fieldCentric = false;

    // ==================== MANUAL LAUNCHER (backup) ====================
    private static final double LAUNCHER_MIN_SPEED = 800;
    private static final double LAUNCHER_MAX_SPEED = 1600;
    private double manualLauncherSpeed = 1200;
    private boolean manualLauncherActive = false;

    // ==================== CONFIGURATION ====================
    private FieldPositions.Alliance alliance = FieldPositions.Alliance.BLUE;
    private FieldPositions.StartPosition startPos = FieldPositions.StartPosition.BLUE_GOAL_SIDE;

    // ==================== DEBOUNCE ====================
    private ElapsedTime bumperDebounce = new ElapsedTime();
    private static final double DEBOUNCE_TIME = 0.25;

    @Override
    public void runOpMode() {
        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Follower Error", e.getMessage());
            follower = null;
        }

        // === CHECK FOR AUTON POSE ===
        boolean hasAutonPose = RobotState.hasValidAutonPose();
        if (hasAutonPose) {
            FieldPositions.Alliance autonAlliance = RobotState.getLastAlliance();
            if (autonAlliance != null) {
                alliance = autonAlliance;
            }
        }

        // === CONFIGURATION PHASE ===
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x) alliance = FieldPositions.Alliance.BLUE;
            if (gamepad1.b) alliance = FieldPositions.Alliance.RED;

            if (gamepad1.dpad_up) {
                startPos = (alliance == FieldPositions.Alliance.BLUE) ?
                        FieldPositions.StartPosition.BLUE_GOAL_SIDE : FieldPositions.StartPosition.RED_GOAL_SIDE;
            }
            if (gamepad1.dpad_down) {
                startPos = (alliance == FieldPositions.Alliance.BLUE) ?
                        FieldPositions.StartPosition.BLUE_PERIMETER_SIDE : FieldPositions.StartPosition.RED_PERIMETER_SIDE;
            }

            telemetry.addLine("=== DynaMOE 19889 TELEOP CONFIG ===");

            if (hasAutonPose) {
                Pose autonPose = RobotState.getAutonEndPose();
                telemetry.addLine(">>> AUTON POSE DETECTED <<<");
                telemetry.addData("Position from Auton", String.format("(%.1f, %.1f, %.1f°)",
                        autonPose.getX(), autonPose.getY(), Math.toDegrees(autonPose.getHeading())));
                telemetry.addData("Pose Age", "%.1f sec", RobotState.getPoseAgeSeconds());
                telemetry.addLine();
            } else {
                telemetry.addLine("(No Auton pose - using manual selection)");
                telemetry.addLine();
            }

            telemetry.addData("Alliance", alliance == FieldPositions.Alliance.BLUE ? "BLUE" : "RED");
            if (!hasAutonPose) {
                telemetry.addData("Start Side", startPos);
            }
            telemetry.addLine();
            telemetry.addLine("X: Blue | B: Red");
            if (!hasAutonPose) {
                telemetry.addLine("D-Pad UP: Goal Side | D-Pad DOWN: Perimeter");
            }
            telemetry.addLine();
            telemetry.addLine("Press START to initialize hardware");
            telemetry.update();

            sleep(10);
        }

        if (isStopRequested()) return;

        // Initialize hardware
        robot = new RobotHardware(telemetry);
        robot.init(hardwareMap, follower, alliance);

        // SET STARTING POSE
        if (follower != null) {
            if (RobotState.hasValidAutonPose()) {
                Pose autonPose = RobotState.getAutonEndPose();
                follower.setStartingPose(autonPose);
                telemetry.addLine("Using pose from Auton:");
                telemetry.addData("Position", String.format("(%.1f, %.1f, %.1f°)",
                        autonPose.getX(), autonPose.getY(), Math.toDegrees(autonPose.getHeading())));
            } else {
                follower.setStartingPose(FieldPositions.getStartPose(startPos));
                telemetry.addLine("Using manual start position:");
                telemetry.addData("Position", startPos);
            }
        }

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        bumperDebounce.reset();

        // IMPORTANT: Tell follower we're in TeleOp mode (stops any active path following)
        // This prevents the robot from moving on its own when TeleOp starts
        // The 'true' parameter enables field-centric drive option
        if (follower != null) {
            follower.startTeleopDrive();
            follower.setTeleOpDrive(0, 0, 0, false);  // Ensure zero movement at start
        }

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {
            follower.update();
            robot.updateSubsystems();

            // Update LauncherAssist every loop (needed for distance/angle calculations)
            if (robot.launcherAssist != null) {
                robot.launcherAssist.update();
            }

            handleLaunchStateMachine();
            handleDriveControls();
            handleIntakeControls();
            handleManualLauncherControls();

            updateTelemetry();
        }

        robot.stopAllSubsystems();
    }

    // ==================== SINGLE-BUTTON LAUNCH STATE MACHINE ====================

    private void handleLaunchStateMachine() {
        // Abort with B button (only during launch sequence)
        if (gamepad1.b && launchState != LaunchState.IDLE) {
            abortLaunch();
            return;
        }

        switch (launchState) {
            case IDLE:
                handleIdleState();
                break;
            case ALIGNING:
                handleAligningState();
                break;
            case FIRING_LEFT:
                handleFiringLeftState();
                break;
            case FIRING_RIGHT:
                handleFiringRightState();
                break;
        }
    }

    private void handleIdleState() {
        // Right Bumper starts launch sequence
        if (gamepad1.right_bumper && bumperDebounce.seconds() > DEBOUNCE_TIME) {
            if (robot.launcherAssist != null) {
                launchState = LaunchState.ALIGNING;
                robot.launcherAssist.resetPID();
                launchTimer.reset();
                bumperDebounce.reset();
            }
        }
    }

    private void handleAligningState() {
        // Spin up launcher to auto-calculated velocity
        double targetVelocity = robot.launcherAssist.getRecommendedVelocity();
        robot.launcher.setTargetVelocity(targetVelocity, targetVelocity - 50);

        // Check if aligned AND launcher ready
        boolean aligned = robot.launcherAssist.isAligned();
        boolean ready = robot.launcher.isReady();

        if (aligned && ready) {
            // Transition to firing
            launchState = LaunchState.FIRING_LEFT;
            robot.launcher.startFeed(LauncherSide.LEFT);
            launchTimer.reset();
        }

        // Timeout protection
        if (launchTimer.seconds() > LAUNCH_TIMEOUT_SECONDS) {
            abortLaunch();
        }
    }

    private void handleFiringLeftState() {
        // Wait for left feed to complete, then start right
        if (!robot.launcher.isFeeding() && launchTimer.seconds() > FEED_DELAY_SECONDS) {
            launchState = LaunchState.FIRING_RIGHT;
            robot.launcher.startFeed(LauncherSide.RIGHT);
            launchTimer.reset();
        }
    }

    private void handleFiringRightState() {
        // Wait for right feed to complete, then return to idle
        if (!robot.launcher.isFeeding() && launchTimer.seconds() > FEED_DELAY_SECONDS) {
            // Launch complete - stop launcher and return to idle
            robot.launcher.stop();
            launchState = LaunchState.IDLE;
        }
    }

    private void abortLaunch() {
        robot.launcher.stop();
        launchState = LaunchState.IDLE;
        bumperDebounce.reset();
    }

    // ==================== DRIVE CONTROLS ====================

    private void handleDriveControls() {
        double y = -gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;  // Negated for Pedro Pathing convention
        double rx;

        // Use auto-align rotation during ALIGNING state
        if (launchState == LaunchState.ALIGNING && robot.launcherAssist != null) {
            rx = -robot.launcherAssist.getRotationPower();  // Negated for Pedro Pathing convention
        } else {
            rx = -gamepad1.right_stick_x;  // Negated for Pedro Pathing convention
        }

        // Field-centric toggle
        if (gamepad1.dpad_left) fieldCentric = false;
        if (gamepad1.dpad_right) fieldCentric = true;

        // Use Pedro Pathing's teleop drive - it handles field-centric internally
        follower.setTeleOpDrive(y, x, rx, fieldCentric);
    }

    // ==================== INTAKE CONTROLS ====================

    private void handleIntakeControls() {
        // Disable intake during launch sequence to prevent interference
        if (launchState != LaunchState.IDLE) return;

        if (gamepad1.right_trigger > 0.5) robot.intake.intake();
        else if (gamepad1.left_trigger > 0.5) robot.intake.outtake();
        else robot.intake.stop();
    }

    // ==================== MANUAL LAUNCHER CONTROLS (BACKUP) ====================

    private void handleManualLauncherControls() {
        // Only allow manual control when not in launch sequence
        if (launchState != LaunchState.IDLE) return;

        // Gamepad2 D-pad adjusts manual speed
        if (gamepad2.dpad_up) {
            manualLauncherSpeed = Math.min(manualLauncherSpeed + 50, LAUNCHER_MAX_SPEED);
            sleep(150);  // Debounce
        } else if (gamepad2.dpad_down) {
            manualLauncherSpeed = Math.max(manualLauncherSpeed - 50, LAUNCHER_MIN_SPEED);
            sleep(150);  // Debounce
        }

        // A: Manual spin up
        if (gamepad1.a) {
            robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            manualLauncherActive = true;
        }

        // Manual feed (only when launcher active and ready)
        if (manualLauncherActive && robot.launcher.isReady()) {
            if (gamepad1.x) robot.launcher.startFeed(LauncherSide.LEFT);
            if (gamepad1.y) robot.launcher.startFeed(LauncherSide.RIGHT);
        }
    }

    // ==================== TELEMETRY ====================

    private void updateTelemetry() {
        telemetry.addLine("=== DYNAMOE 19889 TELEOP ===");

        // Launch state prominently displayed
        telemetry.addLine();
        switch (launchState) {
            case IDLE:
                telemetry.addLine("[ IDLE ] - RB to launch");
                break;
            case ALIGNING:
                telemetry.addLine(">>> ALIGNING... <<<");
                telemetry.addData("  Aligned", robot.launcherAssist.isAligned() ? "YES" : "NO");
                telemetry.addData("  Launcher Ready", robot.launcher.isReady() ? "YES" : "NO");
                break;
            case FIRING_LEFT:
                telemetry.addLine(">>> FIRING LEFT <<<");
                break;
            case FIRING_RIGHT:
                telemetry.addLine(">>> FIRING RIGHT <<<");
                break;
        }
        telemetry.addLine();

        // Position and targeting info
        if (robot.launcherAssist != null) {
            telemetry.addData("Distance to Goal", "%.1f in", robot.launcherAssist.getDistanceToGoal());
            telemetry.addData("Angle Error", "%.1f°", robot.launcherAssist.getAngleErrorDegrees());
            telemetry.addData("Auto Velocity", "%.0f RPM", robot.launcherAssist.getRecommendedVelocity());
        }

        telemetry.addLine();
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getHeading()));

        // Launcher status
        telemetry.addLine();
        double[] vel = robot.launcher.getVelocities();
        telemetry.addData("Launcher L/R", "%.0f / %.0f RPM", vel[0], vel[1]);
        telemetry.addData("Manual Speed", "%.0f RPM", manualLauncherSpeed);

        telemetry.update();
    }
}
