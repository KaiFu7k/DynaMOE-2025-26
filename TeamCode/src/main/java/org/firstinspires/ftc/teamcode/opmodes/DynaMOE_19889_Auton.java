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

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.FieldPositions;
import org.firstinspires.ftc.teamcode.util.FieldPositions.*;
import org.firstinspires.ftc.teamcode.util.MotifDetector;
import org.firstinspires.ftc.teamcode.util.RobotEnums.*;
import static org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;

/**
 * REFACTORED Autonomous OpMode for DynaMOE Team 19889
 * DECODE Game - Supports both GOAL-side and Perimeter-side starting positions
 *
 * IMPROVEMENTS OVER ORIGINAL:
 * - Modular subsystem architecture (easier to debug and test)
 * - Shared code with TeleOp (consistent behavior)
 * - Better telemetry and logging
 * - Cleaner code organization (~350 lines vs 805 lines)
 * - Easy to add new features
 *
 * MATCH DAY WORKFLOW:
 * 1. Place robot at chosen starting position
 * 2. Select "DynaMOE 19889 Auto [Refactored]" from Driver Hub
 * 3. Press INIT
 * 4. Use D-Pad to select position (UP=Blue Goal, DOWN=Blue Perimeter, LEFT=Red Goal, RIGHT=Red Perimeter)
 * 5. Use Y to cycle MOTIF pattern (GPP/PGP/PPG)
 * 6. Press A to confirm
 * 7. Wait for field START
 */
@Autonomous(name = "DynaMOE 19889 Auto", group = "Autonomous")
public class DynaMOE_19889_Auton extends LinearOpMode {

    // ==================== CONSTANTS ====================

    private static final double LAUNCHER_SPINUP_TIMEOUT = 3.0;  // Max time to wait for launcher spinup
    private static final int ARTIFACTS_TO_SCORE = 3;

    // ==================== HARDWARE & SUBSYSTEMS ====================

    private RobotHardware robot;
    private Follower follower;
    private MotifDetector motifDetector;

    // ==================== CONFIGURATION ====================

    private StartPosition startPosition = StartPosition.BLUE_GOAL_SIDE;
    private Alliance alliance = Alliance.BLUE;
    private boolean configurationConfirmed = false;

    // ==================== STATE TRACKING ====================

    private int artifactsScored = 0;
    private ElapsedTime launcherSpinupTimer = new ElapsedTime();

    // ==================== MAIN AUTONOMOUS ====================

    @Override
    public void runOpMode() {
        // Initialize robot hardware and subsystems
        robot = new RobotHardware(telemetry);
        robot.init(hardwareMap);

        // Initialize MOTIF detector
        motifDetector = new MotifDetector(telemetry);
        motifDetector.init(hardwareMap);

        // Initialize Pedro Pathing
        follower = Constants.createFollower(hardwareMap);

        // Configuration selection during INIT phase
        configureAutonomous();

        // Set starting pose
        Pose startPose = FieldPositions.getStartPose(startPosition);
        follower.setStartingPose(startPose);

        telemetry.addLine("=== READY TO START ===");
        telemetry.addData("Position", startPosition);
        telemetry.addData("Status", configurationConfirmed ? "CONFIRMED" : "Ready");
        telemetry.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
            startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Detect MOTIF
        Motif detectedMotif = motifDetector.detectMotif();

        robot.logger.info("Autonomous", "Starting autonomous");
        robot.logger.info("Autonomous", "Position: " + startPosition);
        robot.logger.info("Autonomous", "MOTIF: " + detectedMotif);

        telemetry.addLine("=== STARTING AUTONOMOUS ===");
        telemetry.addData("Position", startPosition);
        telemetry.addData("Detected MOTIF", detectedMotif);
        telemetry.update();
        sleep(500);

        // Execute autonomous based on starting position
        if (FieldPositions.isGoalSide(startPosition)) {
            executeGoalSideAuto(detectedMotif);
        } else {
            executePerimeterSideAuto(detectedMotif);
        }

        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Artifacts Scored", artifactsScored);
        telemetry.update();

        robot.logger.info("Autonomous", "Complete! Scored " + artifactsScored + " artifacts");

        // Clean up
        motifDetector.close();
    }

    // ==================== CONFIGURATION ====================

    private void configureAutonomous() {
        while (!isStarted() && !isStopRequested()) {
            // Position selection using D-Pad
            if (gamepad1.dpad_up) {
                startPosition = StartPosition.BLUE_GOAL_SIDE;
                configurationConfirmed = false;
                sleep(200);
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

            // MOTIF selection (Y button cycles)
            if (gamepad1.y) {
                motifDetector.cycleMotif();
                sleep(200);
            }

            // Confirm selection with A button
            if (gamepad1.a) {
                configurationConfirmed = true;
                sleep(200);
            }

            // Update alliance
            alliance = FieldPositions.getAlliance(startPosition);

            // Display configuration
            displayConfiguration();

            sleep(50);
        }
    }

    private void displayConfiguration() {
        telemetry.addLine("=== DynaMOE 19889 AUTON CONFIG [REFACTORED] ===");
        telemetry.addLine();
        telemetry.addData("Selected Position", startPosition);
        telemetry.addData("Status", configurationConfirmed ? "✓ CONFIRMED" : "Press A to confirm");
        telemetry.addLine();
        telemetry.addData("MOTIF (manual)", motifDetector.getCurrentMotif());
        telemetry.addLine();
        telemetry.addData("Left Slot", robot.artifactManager.getSlotContents(LauncherSide.LEFT));
        telemetry.addData("Right Slot", robot.artifactManager.getSlotContents(LauncherSide.RIGHT));
        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("D-Pad UP:    Blue Goal Side");
        telemetry.addLine("D-Pad DOWN:  Blue Perimeter Side");
        telemetry.addLine("D-Pad LEFT:  Red Goal Side");
        telemetry.addLine("D-Pad RIGHT: Red Perimeter Side");
        telemetry.addLine();
        telemetry.addLine("Y: Cycle MOTIF (GPP/PGP/PPG)");
        telemetry.addLine("A: CONFIRM Selection");
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
    }

    // ==================== AUTONOMOUS SEQUENCES ====================

    /**
     * Autonomous routine for GOAL-SIDE starting position
     * 1. Drive to LAUNCH ZONE
     * 2. Score all preloaded artifacts
     * 3. Move off LAUNCH LINE to earn LEAVE points
     */
    private void executeGoalSideAuto(Motif motif) {
        robot.logger.info("Autonomous", "Executing GOAL-SIDE routine");

        // Step 1: Navigate from starting position to LAUNCH ZONE
        Pose launchPose = FieldPositions.getLaunchPose(alliance);
        moveToPosition(launchPose, "Moving to LAUNCH ZONE");

        // Step 2: Score all artifacts
        scoreAllArtifacts(motif);

        // Step 3: Exit LAUNCH LINE to score LEAVE points
        Pose leavePose = FieldPositions.getLeavePose(alliance);
        moveToPosition(leavePose, "Leaving LAUNCH LINE");

        // Clean up
        robot.launcher.stop();
    }

    /**
     * Autonomous routine for PERIMETER-SIDE starting position
     * 1. Score artifacts (already on LAUNCH LINE)
     * 2. Move off LAUNCH LINE to earn LEAVE points
     */
    private void executePerimeterSideAuto(Motif motif) {
        robot.logger.info("Autonomous", "Executing PERIMETER-SIDE routine");

        telemetry.addLine("Starting PERIMETER-SIDE Autonomous");
        telemetry.addLine("Already in LAUNCH ZONE!");
        telemetry.update();

        // Step 1: Score artifacts
        scoreAllArtifacts(motif);

        // Step 2: Exit LAUNCH LINE to score LEAVE points
        Pose leavePose = FieldPositions.getLeavePose(alliance);
        moveToPosition(leavePose, "Leaving LAUNCH LINE");

        // Clean up
        robot.launcher.stop();
    }

    // ==================== MOVEMENT ====================

    private void moveToPosition(Pose targetPose, String description) {
        robot.logger.info("Movement", description);

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
            telemetry.update();

            sleep(10);
        }

        robot.logger.info("Movement", description + " complete");
    }

    // ==================== LAUNCHER CONTROL ====================

    private void scoreAllArtifacts(Motif motif) {
        robot.logger.info("Scoring", "Starting artifact scoring");

        telemetry.addLine("=== SCORING ARTIFACTS ===");
        telemetry.addData("Target MOTIF", motif);
        telemetry.update();

        // Spin up launchers
        if (!spinUpLaunchers()) {
            robot.logger.error("Scoring", "Launcher failed to spin up!");
            telemetry.addLine("ERROR: Launcher failed to spin up!");
            telemetry.update();
            return;
        }

        // Get pattern sequence
        ArtifactColor[] pattern = robot.artifactManager.getPatternSequence(motif);

        robot.logger.info("Scoring", "Pattern: " + java.util.Arrays.toString(pattern));

        // Launch artifacts in pattern order
        for (int i = 0; i < pattern.length; i++) {
            ArtifactColor targetColor = pattern[i];

            // Find which side has this color
            LauncherSide side = robot.artifactManager.findArtifactSide(targetColor);

            if (side == null) {
                robot.logger.error("Scoring", "Could not find " + targetColor + " artifact!");
                telemetry.addLine("ERROR: Could not find " + targetColor);
                telemetry.update();
                break;
            }

            robot.logger.info("Scoring", "Position " + (i+1) + ": " + targetColor + " from " + side);

            // Launch the artifact
            scoreArtifact(side, targetColor, i + 1);
        }

        robot.logger.info("Scoring", "Scored " + artifactsScored + " artifacts");
    }

    private boolean spinUpLaunchers() {
        robot.logger.info("Launcher", "Spinning up launchers");

        telemetry.addLine("Spinning up launchers...");
        telemetry.update();

        // Start launchers (close shot)
        robot.launcher.spinUp(true);

        launcherSpinupTimer.reset();

        // Wait for launchers to reach minimum velocity
        while (opModeIsActive() && launcherSpinupTimer.seconds() < LAUNCHER_SPINUP_TIMEOUT) {
            double[] velocities = robot.launcher.getVelocities();

            telemetry.addLine("Spinning up launchers...");
            telemetry.addData("Left Velocity", "%.0f RPM", velocities[0]);
            telemetry.addData("Right Velocity", "%.0f RPM", velocities[1]);
            telemetry.addData("Time", "%.1f / %.1f", launcherSpinupTimer.seconds(), LAUNCHER_SPINUP_TIMEOUT);
            telemetry.update();

            if (robot.launcher.isReady()) {
                robot.logger.info("Launcher", "Launchers ready!");
                return true;
            }

            sleep(50);
        }

        // Timeout
        robot.logger.warning("Launcher", "Spinup timeout!");
        return false;
    }

    private void scoreArtifact(LauncherSide side, ArtifactColor color, int artifactNum) {
        // Remove artifact from slot
        ArtifactColor removedColor = robot.artifactManager.removeNext(side);

        if (removedColor != color) {
            robot.logger.warning("Scoring", "Expected " + color + " but got " + removedColor);
        }

        telemetry.addData("Scoring Artifact", artifactNum + " / " + ARTIFACTS_TO_SCORE);
        telemetry.addData("Color", color);
        telemetry.addData("Using", side);
        telemetry.update();

        // Feed artifact (blocking)
        robot.launcher.feed(side, this::opModeIsActive);

        artifactsScored++;

        robot.logger.info("Scoring", "Artifact " + artifactNum + " scored: " + color);

        // Small delay between shots
        sleep(200);
    }
}
