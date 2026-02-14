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
 * 2. FIRING: When aligned and ready, fires L, R, L, R (2 per side to guarantee all 3 launch)
 * 3. Returns to IDLE automatically
 */
@TeleOp(name = "DynaMOE 19889 TeleOp", group = "TeleOp")
public class DynaMOE_19889_TeleOp extends LinearOpMode {

    // ==================== HARDWARE ====================
    private RobotHardware robot;
    private Follower follower;

    // ==================== LAUNCH STATE MACHINE ====================
    private enum LaunchState {
        IDLE,           // Normal driving, launcher off
        ALIGNING,       // Auto-align active, spinning up launcher (aligns for current firingSide)
        FIRING          // Feeding the current side
    }
    // Firing sequence: L, R, L, R (indices 0-3)
    private static final LauncherSide[] FIRE_SEQUENCE = {
        LauncherSide.LEFT, LauncherSide.RIGHT, LauncherSide.LEFT, LauncherSide.RIGHT
    };
    private LaunchState launchState = LaunchState.IDLE;
    private int fireStep = 0;              // Current index in FIRE_SEQUENCE (0-3)
    private ElapsedTime launchTimer = new ElapsedTime();
    private static final double LAUNCH_TIMEOUT_SECONDS = 2.5;  // Max time in ALIGNING state
    private static final double MAX_ROTATION_DEGREES = 400;    // Abort if robot spins this much total
    private double totalRotationDegrees = 0;
    private double lastHeadingForRotation = 0;

    // ==================== DRIVE STATE ====================
    private boolean fieldCentric = false;

    // ==================== MANUAL LAUNCHER (backup) ====================
    private static final double LAUNCHER_MIN_SPEED = 800;
    private static final double LAUNCHER_MAX_SPEED = 1600;
    private double manualLauncherSpeed = 1200;
    private boolean manualLauncherActive = false;

    // ==================== CONFIGURATION ====================
    private FieldPositions.Alliance alliance = FieldPositions.Alliance.BLUE;

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
            // Alliance selection (only matters for practice mode — auton mode inherits alliance)
            if (!hasAutonPose) {
                if (gamepad1.x) alliance = FieldPositions.Alliance.BLUE;
                if (gamepad1.b) alliance = FieldPositions.Alliance.RED;
            }

            telemetry.addLine("=== DynaMOE 19889 TELEOP CONFIG ===");
            telemetry.addLine();

            if (hasAutonPose) {
                Pose autonPose = RobotState.getAutonEndPose();
                telemetry.addLine(">>> MATCH MODE (Auton pose detected) <<<");
                telemetry.addData("Position from Auton", String.format("(%.1f, %.1f, %.1f°)",
                        autonPose.getX(), autonPose.getY(), Math.toDegrees(autonPose.getHeading())));
                telemetry.addData("Alliance", alliance == FieldPositions.Alliance.BLUE ? "BLUE" : "RED");
                telemetry.addData("Pose Age", "%.1f sec", RobotState.getPoseAgeSeconds());
            } else {
                Pose practicePose = FieldPositions.getPerimeterStartPose(alliance);
                telemetry.addLine(">>> PRACTICE MODE (No Auton) <<<");
                telemetry.addData("Alliance", alliance == FieldPositions.Alliance.BLUE ? "BLUE" : "RED");
                telemetry.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
                        practicePose.getX(), practicePose.getY(), Math.toDegrees(practicePose.getHeading())));
                telemetry.addLine();
                telemetry.addLine("X: Blue | B: Red");
                telemetry.addLine("Place robot at PERIMETER START position");
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
                telemetry.addLine("Match mode — using pose from Auton:");
                telemetry.addData("Position", String.format("(%.1f, %.1f, %.1f°)",
                        autonPose.getX(), autonPose.getY(), Math.toDegrees(autonPose.getHeading())));
            } else {
                Pose practicePose = FieldPositions.getPerimeterStartPose(alliance);
                follower.setStartingPose(practicePose);
                telemetry.addLine("Practice mode — using perimeter start:");
                telemetry.addData("Position", String.format("(%.1f, %.1f, %.1f°)",
                        practicePose.getX(), practicePose.getY(), Math.toDegrees(practicePose.getHeading())));
            }
        }

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();
        bumperDebounce.reset();

        // ==================== MAIN LOOP ====================
        while (opModeIsActive()) {
            follower.update();          // Update odometry (motor commands overwritten by handleDriveControls)
            robot.updateSubsystems();   // Updates launcher, launcherAssist, etc.

            handleDriveControls();
            handleLaunchStateMachine();
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
            case FIRING:
                handleFiringState();
                break;
        }
    }

    private void handleIdleState() {
        // Right Bumper starts launch sequence
        if (gamepad1.right_bumper && bumperDebounce.seconds() > DEBOUNCE_TIME) {
            if (robot.launcherAssist != null) {
                fireStep = 0;
                startAligningForCurrentStep();
                bumperDebounce.reset();
            }
        }
    }

    /** Transition to ALIGNING state for the current fireStep's launcher side */
    private void startAligningForCurrentStep() {
        launchState = LaunchState.ALIGNING;
        robot.launcherAssist.setActiveSide(FIRE_SEQUENCE[fireStep]);
        robot.launcherAssist.resetPID();
        robot.intake.intake();  // Keep intake running to seat artifacts against feeders
        launchTimer.reset();
        totalRotationDegrees = 0;
        lastHeadingForRotation = Math.toDegrees(follower.getPose().getHeading());
    }

    private void handleAligningState() {
        // Spin up launcher to auto-calculated velocity
        double targetVelocity = robot.launcherAssist.getRecommendedVelocity();
        robot.launcher.setTargetVelocity(targetVelocity, targetVelocity - 50);

        // Check if aligned AND launcher ready
        boolean aligned = robot.launcherAssist.isAligned();
        boolean ready = robot.launcher.isReady();

        if (aligned && ready) {
            // Fire the current side
            launchState = LaunchState.FIRING;
            robot.launcher.startFeed(FIRE_SEQUENCE[fireStep]);
        }

        // Track total rotation to detect spinning
        double currentHeading = Math.toDegrees(follower.getPose().getHeading());
        double headingDelta = currentHeading - lastHeadingForRotation;
        // Normalize delta to [-180, 180]
        while (headingDelta > 180) headingDelta -= 360;
        while (headingDelta < -180) headingDelta += 360;
        totalRotationDegrees += Math.abs(headingDelta);
        lastHeadingForRotation = currentHeading;

        // Timeout or excessive rotation protection
        if (launchTimer.seconds() > LAUNCH_TIMEOUT_SECONDS || totalRotationDegrees > MAX_ROTATION_DEGREES) {
            abortLaunch();
        }
    }

    private void handleFiringState() {
        // Wait for current feed to complete
        if (!robot.launcher.isFeeding()) {
            fireStep++;
            if (fireStep >= FIRE_SEQUENCE.length) {
                // All shots fired — done
                robot.launcher.stop();
                launchState = LaunchState.IDLE;
            } else {
                // Re-align for the next side, then fire
                startAligningForCurrentStep();
            }
        }
    }

    private void abortLaunch() {
        robot.launcher.stop();
        launchState = LaunchState.IDLE;
        bumperDebounce.reset();
    }

    // ==================== DRIVE CONTROLS ====================

    private void handleDriveControls() {
        double y, x, rx;

        if (launchState == LaunchState.ALIGNING && robot.launcherAssist != null) {
            // Auto-rotate only, no translation
            y = 0; x = 0;
            rx = robot.launcherAssist.getRotationPower();
        } else if (launchState != LaunchState.IDLE) {
            // Firing — hold still
            y = 0; x = 0; rx = 0;
        } else {
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x * 1.1;
            rx = gamepad1.right_stick_x;
        }

        // Field-centric toggle
        if (gamepad1.dpad_left) fieldCentric = false;
        if (gamepad1.dpad_right) fieldCentric = true;

        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;

        if (fieldCentric) {
            double botHeading = follower.getHeading();
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            leftFrontPower = (rotY + rotX + rx) / denominator;
            leftBackPower = (rotY - rotX + rx) / denominator;
            rightFrontPower = (rotY - rotX - rx) / denominator;
            rightBackPower = (rotY + rotX - rx) / denominator;
        } else {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            leftFrontPower = (y + x + rx) / denominator;
            leftBackPower = (y - x + rx) / denominator;
            rightFrontPower = (y - x - rx) / denominator;
            rightBackPower = (y + x - rx) / denominator;
        }

        robot.drivetrain.setPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    // ==================== INTAKE CONTROLS ====================

    private void handleIntakeControls() {
        // Intake is managed by launch sequence when not IDLE
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

        // A: Manual spin up | B: Manual stop
        if (gamepad1.a) {
            robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            manualLauncherActive = true;
        }
        if (gamepad1.b && manualLauncherActive) {
            robot.launcher.stop();
            manualLauncherActive = false;
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
                telemetry.addLine(">>> ALIGNING " + FIRE_SEQUENCE[fireStep] + " " + (fireStep + 1) + "/4 <<<");
                telemetry.addData("  Aligned", robot.launcherAssist.isAligned() ? "YES" : "NO");
                telemetry.addData("  Launcher Ready", robot.launcher.isReady() ? "YES" : "NO");
                telemetry.addData("  Angle Error", "%.1f°", robot.launcherAssist.getAngleErrorDegrees());
                telemetry.addData("  Target Angle", "%.1f°", robot.launcherAssist.getTargetAngleDegrees());
                telemetry.addData("  Current Heading", "%.1f°", robot.launcherAssist.getCurrentHeadingDegrees());
                telemetry.addData("  Rotation Power", "%.2f", robot.launcherAssist.getLastRotationPower());
                telemetry.addData("  Distance", "%.1f in", robot.launcherAssist.getDistanceToGoal());
                break;
            case FIRING:
                telemetry.addLine(">>> FIRING " + FIRE_SEQUENCE[fireStep] + " " + (fireStep + 1) + "/4 <<<");
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
        Pose currentPose = follower.getPose();
        telemetry.addData("Position", String.format("(%.1f, %.1f) @ %.1f°",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));

        // Launcher status
        telemetry.addLine();
        double[] vel = robot.launcher.getVelocities();
        telemetry.addData("Launcher L/R", "%.0f / %.0f RPM", vel[0], vel[1]);
        telemetry.addData("Manual Speed", "%.0f RPM", manualLauncherSpeed);

        telemetry.update();
    }
}
