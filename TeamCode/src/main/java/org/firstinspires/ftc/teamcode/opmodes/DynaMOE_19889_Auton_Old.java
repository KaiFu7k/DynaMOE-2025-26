/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.opmodes;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import java.util.ArrayList;
import java.util.List;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


/**
 * Autonomous OpMode for DynaMOE Team 19889
 * DECODE Game - Supports both GOAL-side and Perimeter-side starting positions
 *
 * MATCH DAY WORKFLOW:
 * 1. Place robot at chosen starting position
 * 2. Select "DynaMOE 19889 Auto" from Driver Hub
 * 3. Press INIT
 * 4. Use D-Pad to select position (UP=Blue Goal, DOWN=Blue Perimeter, LEFT=Red Goal, RIGHT=Red Perimeter)
 * 5. Press A to confirm
 * 6. Wait for field START
 */
@Disabled
@Autonomous(name = "DynaMOE 19889 Auto Old", group = "Autonomous")
public class DynaMOE_19889_Auton_Old extends LinearOpMode {

    // ==================== CONSTANTS ====================

    // Launcher velocity constants (tuned for DECODE artifacts - match TeleOp values)
    private static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1200;  // RPM for close shots
    private static final double LAUNCHER_CLOSE_MIN_VELOCITY = 1175;     // Minimum acceptable RPM
    private static final double LAUNCHER_FAR_TARGET_VELOCITY = 1350;    // RPM for far shots (not used in auto)
    private static final double LAUNCHER_FAR_MIN_VELOCITY = 1325;       // Minimum for far shots

    // Feeder servo timing
    private static final double FEED_TIME_SECONDS = 0.80;  // Time to feed one artifact
    private static final double STOP_SPEED = 0.0;
    private static final double FULL_SPEED = 1.0;

    // Diverter servo positions (directs intake to left or right launcher)
    private static final double LEFT_POSITION = 0.2962;
    private static final double RIGHT_POSITION = 0;

    // Autonomous-specific constants
    private static final double LAUNCHER_SPINUP_TIMEOUT = 3.0;  // Max time to wait for launcher spinup
    private static final double ARTIFACTS_TO_SCORE = 3;         // Number of preloaded artifacts

    // ==================== FIELD COORDINATES ====================
    // NOTE: All coordinates in inches, angles in radians
    // COORDINATE SYSTEM (2025-26 DECODE season):
    // - Origin (0, 0) is at BOTTOM-LEFT corner of field
    // - X: 0 to 144 inches (left to right)
    // - Y: 0 to 144 inches (bottom to top)
    // - Heading: 0° = right (+X), 90° = up (+Y), 180° = left, 270° = down

    // Starting positions - where robot begins autonomous (TUNE THESE FOR YOUR FIELD!)
    private static final Pose BLUE_GOAL_SIDE_START = new Pose(36, 132, Math.toRadians(90));       // Blue, near goal
    private static final Pose RED_GOAL_SIDE_START = new Pose(108, 132, Math.toRadians(90));       // Red, near goal
    private static final Pose BLUE_PERIMETER_START = new Pose(12, 108, Math.toRadians(0));        // Blue, on perimeter
    private static final Pose RED_PERIMETER_START = new Pose(132, 108, Math.toRadians(180));      // Red, on perimeter

    // Launch positions - where robot shoots artifacts (must be inside LAUNCH ZONE)
    private static final Pose BLUE_LAUNCH_POSE = new Pose(48, 96, Math.toRadians(135));           // Blue launch spot
    private static final Pose RED_LAUNCH_POSE = new Pose(96, 96, Math.toRadians(45));             // Red launch spot

    // Leave positions - final position to score LEAVE points (must be off LAUNCH LINE)
    private static final Pose BLUE_LEAVE_POSE = new Pose(36, 72, Math.toRadians(90));
    private static final Pose RED_LEAVE_POSE = new Pose(108, 72, Math.toRadians(90));

    // ==================== CONFIGURATION ENUMS ====================

    /**
     * Four possible starting positions (combines alliance + side)
     * Selected via D-Pad during INIT phase
     */
    public enum StartPosition {
        BLUE_GOAL_SIDE,       // Blue alliance, near the GOAL (D-Pad UP)
        BLUE_PERIMETER_SIDE,  // Blue alliance, on perimeter LAUNCH LINE (D-Pad DOWN)
        RED_GOAL_SIDE,        // Red alliance, near the GOAL (D-Pad LEFT)
        RED_PERIMETER_SIDE    // Red alliance, on perimeter LAUNCH LINE (D-Pad RIGHT)
    }

    /**
     * Alliance color - automatically determined from StartPosition
     */
    public enum Alliance {
        RED,
        BLUE
    }

    /**
     * Which launcher slot to use for feeding artifacts
     */
    public enum LauncherSide {
        LEFT,
        RIGHT,
        BOTH    // Not currently used
    }

    /**
     * Artifact colors for DECODE game
     */
    public enum ArtifactColor {
        PURPLE,
        GREEN
    }

    /**
     * MOTIF patterns displayed on OBELISK
     * Determines the order artifacts must be launched
     */
    public enum Motif {
        GPP,  // Green, Purple, Purple (launch order)
        PGP,  // Purple, Green, Purple
        PPG   // Purple, Purple, Green
    }

    /**
     * Queue data structure for tracking artifacts in each launcher slot
     * First artifact added is first to launch (FIFO)
     */
    private static class ArtifactSlot {
        List<ArtifactColor> artifacts = new ArrayList<>();

        void add(ArtifactColor color) {
            artifacts.add(color);
        }

        ArtifactColor removeNext() {
            if (!artifacts.isEmpty()) {
                return artifacts.remove(0);  // Remove from front of queue
            }
            return null;
        }

        boolean isEmpty() {
            return artifacts.isEmpty();
        }

        int size() {
            return artifacts.size();
        }
    }

    // ==================== CONFIGURATION VARIABLES ====================

    // Position configuration - selected during INIT phase via gamepad
    private StartPosition startPosition = StartPosition.BLUE_GOAL_SIDE;
    private Alliance alliance = Alliance.BLUE;
    private boolean configurationConfirmed = false;  // Tracks if driver pressed A to confirm

    // Artifact slots - MODIFY configurePreload() to match your actual robot setup
    // Default configuration: LEFT=[Purple, Purple], RIGHT=[Green]
    private ArtifactSlot leftSlot = new ArtifactSlot();
    private ArtifactSlot rightSlot = new ArtifactSlot();

    // MOTIF detection - currently manual (Y button), will be replaced with AprilTag vision
    private Motif detectedMotif = Motif.GPP;

    // ==================== HARDWARE ====================

    // Pedro Pathing follower for autonomous navigation
    private Follower follower;

    // Drivetrain motors (mecanum drive)
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Launcher motors (DcMotorEx for velocity control)
    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;

    // Feeder servos (feed artifacts into launchers)
    private CRServo leftFeeder;
    private CRServo rightFeeder;

    // Intake and artifact routing
    private DcMotor intake;
    private Servo diverter;  // Routes artifacts to left or right launcher

    // Timers for controlling actions
    private ElapsedTime leftFeederTimer = new ElapsedTime();
    private ElapsedTime rightFeederTimer = new ElapsedTime();
    private ElapsedTime launcherSpinupTimer = new ElapsedTime();

    // Match state tracking
    private int artifactsScored = 0;

    // ==================== MAIN AUTONOMOUS ====================

    @Override
    public void runOpMode() {
        // Initialize hardware
        initializeHardware();

        // Configure preloaded artifacts
        configurePreload();

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);

        // Configuration selection during INIT phase
        configureAutonomous();

        // Set starting pose
        Pose startPose = getStartPose();
        follower.setStartingPose(startPose);

        telemetry.addLine("=== READY TO START ===");
        telemetry.addData("Position", startPosition);
        telemetry.addData("Status", configurationConfirmed ? "CONFIRMED" : "Ready");
        telemetry.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
            startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Detect MOTIF (placeholder - will be replaced with vision)
        detectedMotif = detectMotif();

        telemetry.addLine("=== STARTING AUTONOMOUS ===");
        telemetry.addData("Position", startPosition);
        telemetry.addData("Detected MOTIF", detectedMotif);
        telemetry.update();
        sleep(500);

        // Execute autonomous based on starting position
        if (startPosition == StartPosition.BLUE_GOAL_SIDE || startPosition == StartPosition.RED_GOAL_SIDE) {
            executeGoalSideAuto();
        } else {
            executePerimeterSideAuto();
        }

        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Artifacts Scored", artifactsScored);
        telemetry.update();
    }

    // ==================== HARDWARE INITIALIZATION ====================

    private void initializeHardware() {
        // Initialize drivetrain
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Initialize launchers
        leftLauncher = hardwareMap.get(DcMotorEx.class, "left_launcher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "right_launcher");

        // Initialize feeders
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");

        // Initialize intake and diverter
        intake = hardwareMap.get(DcMotor.class, "intake");
        diverter = hardwareMap.get(Servo.class, "diverter");

        // Set motor directions (match TeleOp)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        leftLauncher.setZeroPowerBehavior(BRAKE);
        rightLauncher.setZeroPowerBehavior(BRAKE);

        // Set encoder modes
        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configure PIDF for velocity control
        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
            new PIDFCoefficients(300, 0, 0, 10));

        // Initialize servos
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        diverter.setPosition(LEFT_POSITION); // Default to left

        telemetry.addData("Hardware", "Initialized");
        telemetry.update();
    }

    // ==================== CONFIGURATION ====================

    private void configureAutonomous() {
        while (!isStarted() && !isStopRequested()) {
            // Position selection using D-Pad
            if (gamepad1.dpad_up) {
                startPosition = StartPosition.BLUE_GOAL_SIDE;
                configurationConfirmed = false;
                sleep(200); // Debounce
            } else if (gamepad1.dpad_down) {
                startPosition = StartPosition.BLUE_PERIMETER_SIDE;
                configurationConfirmed = false;
                sleep(200);
            } else if (gamepad1.dpad_left) {
                startPosition = StartPosition.RED_GOAL_SIDE;
                configurationConfirmed = false;
                sleep(200);
            } else if (gamepad1.dpad_right) {
                startPosition = StartPosition.RED_PERIMETER_SIDE;
                configurationConfirmed = false;
                sleep(200);
            }

            // MOTIF selection (Y button cycles) - for testing without vision
            if (gamepad1.y) {
                if (detectedMotif == Motif.GPP) {
                    detectedMotif = Motif.PGP;
                } else if (detectedMotif == Motif.PGP) {
                    detectedMotif = Motif.PPG;
                } else {
                    detectedMotif = Motif.GPP;
                }
                sleep(200); // Debounce
            }

            // Confirm selection with A button
            if (gamepad1.a) {
                configurationConfirmed = true;
                sleep(200);
            }

            // Update alliance based on selected position
            if (startPosition == StartPosition.BLUE_GOAL_SIDE || startPosition == StartPosition.BLUE_PERIMETER_SIDE) {
                alliance = Alliance.BLUE;
            } else {
                alliance = Alliance.RED;
            }

            // Display configuration
            telemetry.addLine("=== DynaMOE 19889 AUTON CONFIG ===");
            telemetry.addLine();
            telemetry.addData("Selected Position", startPosition);
            telemetry.addData("Status", configurationConfirmed ? "✓ CONFIRMED" : "Press A to confirm");
            telemetry.addLine();
            telemetry.addData("MOTIF (manual)", detectedMotif);
            telemetry.addLine();
            telemetry.addData("Left Slot", getSlotContents(leftSlot));
            telemetry.addData("Right Slot", getSlotContents(rightSlot));
            telemetry.addLine();
            telemetry.addLine("--- CONTROLS ---");
            telemetry.addLine("D-Pad UP:    Blue Goal Side");
            telemetry.addLine("D-Pad DOWN:  Blue Perimeter Side");
            telemetry.addLine("D-Pad LEFT:  Red Goal Side");
            telemetry.addLine("D-Pad RIGHT: Red Perimeter Side");
            telemetry.addLine();
            telemetry.addLine("Y: Cycle MOTIF (GPP/PGP/PPG)");
            telemetry.addLine("A: CONFIRM Selection");

            // Visual position guide
            telemetry.addLine();
            telemetry.addLine("--- FIELD LAYOUT ---");
            telemetry.addLine("        BLUE GOAL");
            telemetry.addLine("    UP        DOWN");
            telemetry.addLine("  (Goal)  (Perimeter)");
            telemetry.addLine();
            telemetry.addLine("  LEFT      RIGHT");
            telemetry.addLine("  (Goal)  (Perimeter)");
            telemetry.addLine("        RED GOAL");

            telemetry.update();

            sleep(50); // Prevent loop hogging
        }
    }

    /**
     * Configure which artifacts are loaded in each launcher slot
     * IMPORTANT: Modify this to match your physical robot setup before each match!
     *
     * Current default: LEFT=[Purple, Purple], RIGHT=[Green]
     * Order matters - first added is first to launch (FIFO queue)
     */
    private void configurePreload() {
        // Default configuration: 2 Purple + 1 Green
        leftSlot.add(ArtifactColor.PURPLE);   // First purple
        leftSlot.add(ArtifactColor.PURPLE);   // Second purple
        rightSlot.add(ArtifactColor.GREEN);   // Green

        // Alternative configurations (comment out default above and uncomment one below):

        // Option 1: LEFT=[Purple, Green], RIGHT=[Purple]
        // leftSlot.add(ArtifactColor.PURPLE);
        // leftSlot.add(ArtifactColor.GREEN);
        // rightSlot.add(ArtifactColor.PURPLE);

        // Option 2: LEFT=[Green, Purple], RIGHT=[Purple]
        // leftSlot.add(ArtifactColor.GREEN);
        // leftSlot.add(ArtifactColor.PURPLE);
        // rightSlot.add(ArtifactColor.PURPLE);
    }

    private String getSlotContents(ArtifactSlot slot) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < slot.artifacts.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(slot.artifacts.get(i) == ArtifactColor.PURPLE ? "P" : "G");
        }
        sb.append("]");
        return sb.toString();
    }

    private Pose getStartPose() {
        switch (startPosition) {
            case BLUE_GOAL_SIDE:
                return BLUE_GOAL_SIDE_START;
            case BLUE_PERIMETER_SIDE:
                return BLUE_PERIMETER_START;
            case RED_GOAL_SIDE:
                return RED_GOAL_SIDE_START;
            case RED_PERIMETER_SIDE:
                return RED_PERIMETER_START;
            default:
                return BLUE_GOAL_SIDE_START;
        }
    }

    private Pose getLaunchPose() {
        return (alliance == Alliance.BLUE) ? BLUE_LAUNCH_POSE : RED_LAUNCH_POSE;
    }

    private Pose getLeavePose() {
        return (alliance == Alliance.BLUE) ? BLUE_LEAVE_POSE : RED_LEAVE_POSE;
    }

    // ==================== AUTONOMOUS SEQUENCES ====================

    /**
     * Autonomous routine for GOAL-SIDE starting position
     * 1. Drive to LAUNCH ZONE
     * 2. Score all preloaded artifacts
     * 3. Move off LAUNCH LINE to earn LEAVE points
     */
    private void executeGoalSideAuto() {
        telemetry.addLine("Starting GOAL-SIDE Autonomous");
        telemetry.update();

        // Step 1: Navigate from starting position to LAUNCH ZONE
        Pose launchPose = getLaunchPose();
        moveToPosition(launchPose, "Moving to LAUNCH ZONE");

        // Step 2: Spin up launchers and score all artifacts in correct MOTIF order
        scoreAllArtifacts();

        // Step 3: Exit LAUNCH LINE to score LEAVE points (4 points)
        Pose leavePose = getLeavePose();
        moveToPosition(leavePose, "Leaving LAUNCH LINE");

        // Clean up - stop all launcher motors
        stopLaunchers();
    }

    /**
     * Autonomous routine for PERIMETER-SIDE starting position
     * 1. Score artifacts (already on LAUNCH LINE)
     * 2. Move off LAUNCH LINE to earn LEAVE points
     */
    private void executePerimeterSideAuto() {
        telemetry.addLine("Starting PERIMETER-SIDE Autonomous");
        telemetry.addLine("Already in LAUNCH ZONE!");
        telemetry.update();

        // Step 1: We start on LAUNCH LINE - immediately score artifacts
        scoreAllArtifacts();

        // Step 2: Exit LAUNCH LINE to score LEAVE points (4 points)
        Pose leavePose = getLeavePose();
        moveToPosition(leavePose, "Leaving LAUNCH LINE");

        // Clean up - stop all launcher motors
        stopLaunchers();
    }

    // ==================== MOVEMENT ====================

    private void moveToPosition(Pose targetPose, String description) {
        telemetry.addLine(description);
        telemetry.addData("Target", String.format("(%.1f, %.1f, %.1f°)",
            targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading())));
        telemetry.update();

        // Build path
        PathChain path = follower.pathBuilder()
            .addPath(new BezierLine(
                follower.getPose(),
                targetPose
            ))
            .setLinearHeadingInterpolation(
                follower.getPose().getHeading(),
                targetPose.getHeading()
            )
            .build();

        // Follow path
        follower.followPath(path);

        // Update follower until path is complete
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();

            Pose currentPose = follower.getPose();
            telemetry.addLine(description);
            telemetry.addData("Current", String.format("(%.1f, %.1f, %.1f°)",
                currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));
            telemetry.addData("Target", String.format("(%.1f, %.1f, %.1f°)",
                targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading())));
            telemetry.update();

            sleep(10);
        }

        telemetry.addLine(description + " - COMPLETE");
        telemetry.update();
    }

    // ==================== LAUNCHER CONTROL ====================

    private void scoreAllArtifacts() {
        telemetry.addLine("=== SCORING ARTIFACTS ===");
        telemetry.addData("Target MOTIF", detectedMotif);
        telemetry.update();

        // Spin up launchers
        if (!spinUpLaunchers()) {
            telemetry.addLine("ERROR: Launcher failed to spin up!");
            telemetry.update();
            return;
        }

        // Get the pattern sequence for the detected MOTIF
        ArtifactColor[] pattern = getPatternSequence(detectedMotif);

        telemetry.addLine("Launching in pattern order:");
        for (ArtifactColor color : pattern) {
            telemetry.addData("  ", color);
        }
        telemetry.update();
        sleep(1000); // Show pattern briefly

        // Launch artifacts in pattern order
        for (int i = 0; i < pattern.length; i++) {
            ArtifactColor targetColor = pattern[i];

            // Find which side has this color
            LauncherSide side = findArtifactSide(targetColor);

            if (side == null) {
                telemetry.addLine("ERROR: Could not find " + targetColor + " artifact!");
                telemetry.update();
                break;
            }

            telemetry.addLine("Pattern position " + (i + 1) + ": Launch " + targetColor + " from " + side);
            telemetry.update();

            // Launch the artifact
            scoreArtifactByColor(side, targetColor, i + 1);
        }

        telemetry.addData("Artifacts Scored", artifactsScored);
        telemetry.update();
    }

    private boolean spinUpLaunchers() {
        telemetry.addLine("Spinning up launchers...");
        telemetry.update();

        // Start launchers at CLOSE velocity
        leftLauncher.setVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);
        rightLauncher.setVelocity(LAUNCHER_CLOSE_TARGET_VELOCITY);

        launcherSpinupTimer.reset();

        // Wait for launchers to reach minimum velocity
        while (opModeIsActive() && launcherSpinupTimer.seconds() < LAUNCHER_SPINUP_TIMEOUT) {
            double leftVel = leftLauncher.getVelocity();
            double rightVel = rightLauncher.getVelocity();

            telemetry.addLine("Spinning up launchers...");
            telemetry.addData("Left Velocity", "%.0f / %.0f", leftVel, LAUNCHER_CLOSE_MIN_VELOCITY);
            telemetry.addData("Right Velocity", "%.0f / %.0f", rightVel, LAUNCHER_CLOSE_MIN_VELOCITY);
            telemetry.addData("Time", "%.1f / %.1f", launcherSpinupTimer.seconds(), LAUNCHER_SPINUP_TIMEOUT);
            telemetry.update();

            // Check if both launchers are ready
            if (leftVel > LAUNCHER_CLOSE_MIN_VELOCITY && rightVel > LAUNCHER_CLOSE_MIN_VELOCITY) {
                telemetry.addLine("Launchers ready!");
                telemetry.update();
                return true;
            }

            sleep(50);
        }

        // Timeout - return false
        return false;
    }

    private void scoreArtifactByColor(LauncherSide side, ArtifactColor color, int artifactNum) {
        // Remove the artifact from the slot
        ArtifactColor removedColor = null;
        if (side == LauncherSide.LEFT) {
            removedColor = leftSlot.removeNext();
        } else {
            removedColor = rightSlot.removeNext();
        }

        // Verify we got the right color
        if (removedColor != color) {
            telemetry.addLine("WARNING: Expected " + color + " but got " + removedColor);
            telemetry.update();
        }

        telemetry.addData("Scoring Artifact", artifactNum + " / " + (int)ARTIFACTS_TO_SCORE);
        telemetry.addData("Color", color);
        telemetry.addData("Using", side);
        telemetry.addData("Left Slot Remaining", getSlotContents(leftSlot));
        telemetry.addData("Right Slot Remaining", getSlotContents(rightSlot));
        telemetry.update();

        // Activate appropriate feeder
        if (side == LauncherSide.LEFT) {
            leftFeeder.setPower(FULL_SPEED);
            leftFeederTimer.reset();

            // Wait for feed time
            while (opModeIsActive() && leftFeederTimer.seconds() < FEED_TIME_SECONDS) {
                telemetry.addData("Feeding LEFT (" + color + ")", "%.2f / %.2f",
                    leftFeederTimer.seconds(), FEED_TIME_SECONDS);
                telemetry.update();
                sleep(10);
            }

            leftFeeder.setPower(STOP_SPEED);
        } else {
            rightFeeder.setPower(FULL_SPEED);
            rightFeederTimer.reset();

            // Wait for feed time
            while (opModeIsActive() && rightFeederTimer.seconds() < FEED_TIME_SECONDS) {
                telemetry.addData("Feeding RIGHT (" + color + ")", "%.2f / %.2f",
                    rightFeederTimer.seconds(), FEED_TIME_SECONDS);
                telemetry.update();
                sleep(10);
            }

            rightFeeder.setPower(STOP_SPEED);
        }

        artifactsScored++;

        telemetry.addLine("Artifact scored: " + color);
        telemetry.update();

        // Small delay between shots
        sleep(200);
    }

    private void stopLaunchers() {
        leftLauncher.setVelocity(STOP_SPEED);
        rightLauncher.setVelocity(STOP_SPEED);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        telemetry.addLine("Launchers stopped");
        telemetry.update();
    }

    // ==================== MOTIF DETECTION ====================

    /**
     * Detect the MOTIF pattern from the OBELISK
     * TODO: Replace with AprilTag vision detection
     *
     * IMPLEMENTATION PLAN:
     * 1. Initialize camera and AprilTag processor
     * 2. Look for OBELISK tags (IDs 21, 22, 23)
     * 3. Read which tag is highlighted/active
     * 4. Map tag ID to MOTIF pattern
     *
     * For now, uses manually configured value from init (Y button)
     */
    private Motif detectMotif() {
        telemetry.addLine("Detecting MOTIF...");
        telemetry.addData("MOTIF", detectedMotif);
        telemetry.update();

        // TODO: Add AprilTag vision code here
        // VisionPortal portal = new VisionPortal.Builder()...
        // AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()...
        // Detect which of tags 21/22/23 is active -> return corresponding MOTIF

        return detectedMotif;  // Currently returns manually set value
    }

    // ==================== PATTERN MATCHING ====================

    /**
     * Convert MOTIF enum to launch sequence array
     * Returns the order in which artifacts must be launched (first to last)
     */
    private ArtifactColor[] getPatternSequence(Motif motif) {
        switch (motif) {
            case GPP:
                return new ArtifactColor[] {
                    ArtifactColor.GREEN,    // Launch 1st
                    ArtifactColor.PURPLE,   // Launch 2nd
                    ArtifactColor.PURPLE    // Launch 3rd
                };
            case PGP:
                return new ArtifactColor[] {
                    ArtifactColor.PURPLE,   // Launch 1st
                    ArtifactColor.GREEN,    // Launch 2nd
                    ArtifactColor.PURPLE    // Launch 3rd
                };
            case PPG:
                return new ArtifactColor[] {
                    ArtifactColor.PURPLE,   // Launch 1st
                    ArtifactColor.PURPLE,   // Launch 2nd
                    ArtifactColor.GREEN     // Launch 3rd
                };
            default:
                return new ArtifactColor[] {
                    ArtifactColor.GREEN,
                    ArtifactColor.PURPLE,
                    ArtifactColor.PURPLE
                };
        }
    }

    /**
     * Find which launcher slot contains the target artifact color
     * Searches left slot first, then right slot
     * Returns null if color not found (error condition)
     */
    private LauncherSide findArtifactSide(ArtifactColor targetColor) {
        // Check left slot first
        for (ArtifactColor color : leftSlot.artifacts) {
            if (color == targetColor) {
                return LauncherSide.LEFT;
            }
        }

        // Check right slot
        for (ArtifactColor color : rightSlot.artifacts) {
            if (color == targetColor) {
                return LauncherSide.RIGHT;
            }
        }

        // Artifact color not found in either slot - configuration error!
        return null;
    }
}
