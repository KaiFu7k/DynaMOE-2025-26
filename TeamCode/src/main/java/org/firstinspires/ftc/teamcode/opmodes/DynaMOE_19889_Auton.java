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
 * 2. Select "DynaMOE 19889 Auto" from Driver Hub
 * 3. Press INIT
 * 4. Use D-Pad to select position (UP=Blue Goal, DOWN=Blue Perimeter, LEFT=Red Goal, RIGHT=Red Perimeter)
 * 5. Press A to confirm
 * 6. Wait for field START
 *
 * NOTE: MOTIF detection is DISABLED - All artifacts shot into goal regardless of pattern
 */
@Autonomous(name = "DynaMOE 19889 Auto", group = "Autonomous")
public class DynaMOE_19889_Auton extends LinearOpMode {

    // ==================== CONSTANTS ====================

    private static final double LAUNCHER_SPINUP_TIMEOUT = 3.0;  // Max time to wait for launcher spinup
    private static final int ARTIFACTS_TO_SCORE = 3;

    // ==================== HARDWARE & SUBSYSTEMS ====================

    private RobotHardware robot;
    private Follower follower;
    // NOTE: MotifDetector removed - no longer checking patterns

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

        // NOTE: MOTIF detector removed - ignoring patterns, shooting all artifacts

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
        telemetry.addData("Strategy", "SHOOT ALL ARTIFACTS (ignore MOTIF)");
        telemetry.addData("Start Pose", String.format("(%.1f, %.1f, %.1f°)",
            startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading())));
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Update follower once at start to initialize
        follower.update();

        robot.logger.info("Autonomous", "Starting autonomous");
        robot.logger.info("Autonomous", "Position: " + startPosition);
        robot.logger.info("Autonomous", "Strategy: SHOOT ALL (ignore MOTIF)");

        telemetry.addLine("=== STARTING AUTONOMOUS ===");
        telemetry.addData("Position", startPosition);
        telemetry.addData("Strategy", "SHOOT ALL");
        telemetry.update();
       // sleep(500);

        // Execute autonomous based on starting position
        // NOTE: No MOTIF parameter - shooting all artifacts
        if (FieldPositions.isGoalSide(startPosition)) {
            executeGoalSideAuto();
        } else {
            executePerimeterSideAuto();
        }

        // Show final position
        Pose finalPose = follower.getPose();
        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addLine();
        telemetry.addLine("--- FINAL POSITION ---");
        telemetry.addData("X", "%.1f inches", finalPose.getX());
        telemetry.addData("Y", "%.1f inches", finalPose.getY());
        telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(finalPose.getHeading()));
        telemetry.addLine();
        telemetry.addData("Artifacts Scored", artifactsScored);
        telemetry.update();

        robot.logger.info("Autonomous", "Complete! Scored " + artifactsScored + " artifacts");
        robot.logger.info("Autonomous", String.format("Final position: (%.1f, %.1f, %.1f°)",
            finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading())));
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

            // NOTE: MOTIF selection removed - no longer needed

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
        // Get current start pose for display
        Pose startPose = FieldPositions.getStartPose(startPosition);

        telemetry.addLine("=== DynaMOE 19889 AUTON CONFIG ===");
        telemetry.addLine();
        telemetry.addData("Selected Position", startPosition);
        telemetry.addData("Start X", "%.1f inches", startPose.getX());
        telemetry.addData("Start Y", "%.1f inches", startPose.getY());
        telemetry.addData("Start Heading", "%.1f degrees", Math.toDegrees(startPose.getHeading()));
        telemetry.addLine();
        telemetry.addData("Status", configurationConfirmed ? "✓ CONFIRMED" : "Press A to confirm");
        telemetry.addLine();
        telemetry.addData("Strategy", "SHOOT ALL ARTIFACTS");
        telemetry.addData("MOTIF Check", "DISABLED");
        telemetry.addLine();
        telemetry.addData("Artifacts to Score", ARTIFACTS_TO_SCORE);
        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("D-Pad UP:    Blue Goal Side");
        telemetry.addLine("D-Pad DOWN:  Blue Perimeter Side");
        telemetry.addLine("D-Pad LEFT:  Red Goal Side");
        telemetry.addLine("D-Pad RIGHT: Red Perimeter Side");
        telemetry.addLine();
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
     * 1. Move to LAUNCH ZONE (also earns LEAVE points by leaving start area)
     * 2. Launch all artifacts
     * 3. Move to final/parking position
     */
    private void executeGoalSideAuto() {
        robot.logger.info("Autonomous", "Executing GOAL-SIDE routine");

        // Step 1: Navigate from starting position to LAUNCH ZONE
        // This also earns LEAVE points since we're leaving the starting area
        Pose launchPose = FieldPositions.getLaunchPose(alliance);
        robot.logger.info("Autonomous", String.format("Launch target: (%.1f, %.1f, %.1f°)",
            launchPose.getX(), launchPose.getY(), Math.toDegrees(launchPose.getHeading())));
        moveToPosition(launchPose, "Step 1: Moving to LAUNCH ZONE");

        // Step 2: Score all artifacts
        scoreAllArtifacts();

        // Step 3: Move to final/parking position
        Pose finalPose = FieldPositions.getLeavePose(alliance);
        robot.logger.info("Autonomous", String.format("Final target: (%.1f, %.1f, %.1f°)",
            finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading())));
        moveToPosition(finalPose, "Step 3: Moving to FINAL position");

        // Clean up
        robot.launcher.stop();
    }

    /**
     * Autonomous routine for PERIMETER-SIDE starting position
     * 1. Score artifacts (already on LAUNCH LINE)
     * 2. Move off LAUNCH LINE to earn LEAVE points
     */
    private void executePerimeterSideAuto() {
        robot.logger.info("Autonomous", "Executing PERIMETER-SIDE routine");

        telemetry.addLine("Starting PERIMETER-SIDE Autonomous");
        telemetry.addLine("Already in LAUNCH ZONE!");
        telemetry.update();

        // Step 1: Score artifacts (no MOTIF checking)
        scoreAllArtifacts();

        // Step 2: Exit LAUNCH LINE to score LEAVE points
        Pose leavePose = FieldPositions.getLeavePose(alliance);
        moveToPosition(leavePose, "Leaving LAUNCH LINE");

        // Clean up
        robot.launcher.stop();
    }

    // ==================== MOVEMENT ====================

    private void moveToPosition(Pose targetPose, String description) {
        robot.logger.info("Movement", description);

        // Get current pose for logging
        Pose currentPose = follower.getPose();
        robot.logger.info("Movement", String.format("FROM: (%.1f, %.1f, %.1f°)",
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getHeading())));
        robot.logger.info("Movement", String.format("TO: (%.1f, %.1f, %.1f°)",
            targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getHeading())));

        // Build path using a new Pose with same values (avoid reference issues)
        Pose startPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());

        PathChain path = follower.pathBuilder()
            .addPath(new BezierLine(
                startPose,
                targetPose
            ))
            .setLinearHeadingInterpolation(
                startPose.getHeading(),
                targetPose.getHeading()
            )
            .build();

        // Follow path (true = hold end position)
        follower.followPath(path, true);

        // Debug: Check if follower started
        robot.logger.info("Movement", "followPath called, isBusy=" + follower.isBusy());

        // Update follower until path is complete
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
            updatePositionTelemetry(description, targetPose);
        }

        // Debug: Check why loop exited
        robot.logger.info("Movement", "Loop exited, isBusy=" + follower.isBusy() + ", opModeIsActive=" + opModeIsActive());

        // Final position update
        updatePositionTelemetry(description + " - COMPLETE", targetPose);
        robot.logger.info("Movement", description + " complete");
    }

    /**
     * Updates telemetry with current robot position and target
     */
    private void updatePositionTelemetry(String status, Pose targetPose) {
        Pose currentPose = follower.getPose();

        telemetry.addLine("=== ROBOT POSITION ===");
        telemetry.addData("Status", status);
        telemetry.addLine();
        telemetry.addLine("--- CURRENT ---");
        telemetry.addData("X", "%.1f inches", currentPose.getX());
        telemetry.addData("Y", "%.1f inches", currentPose.getY());
        telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentPose.getHeading()));
        telemetry.addLine();
        telemetry.addLine("--- TARGET ---");
        telemetry.addData("X", "%.1f inches", targetPose.getX());
        telemetry.addData("Y", "%.1f inches", targetPose.getY());
        telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(targetPose.getHeading()));
        telemetry.addLine();

        // Calculate distance to target
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);
        telemetry.addData("Distance to Target", "%.1f inches", distance);

        telemetry.update();
    }

    // ==================== LAUNCHER CONTROL ====================

    private void scoreAllArtifacts() {
        robot.logger.info("Scoring", "Starting artifact scoring - SHOOT ALL (ignore MOTIF)");

        telemetry.addLine("=== SCORING ARTIFACTS ===");
        telemetry.addData("Strategy", "SHOOT ALL");
        telemetry.addData("MOTIF Check", "DISABLED");
        telemetry.update();

        // Spin up launchers
        if (!spinUpLaunchers()) {
            robot.logger.error("Scoring", "Launcher failed to spin up!");
            telemetry.addLine("ERROR: Launcher failed to spin up!");
            telemetry.update();
            return;
        }

        // Simple shooting order: LEFT, RIGHT, LEFT
        // This ensures we shoot all 3 preloaded artifacts regardless of color
        LauncherSide[] shootingOrder = {
            LauncherSide.LEFT,
            LauncherSide.RIGHT,
            LauncherSide.LEFT
        };

        robot.logger.info("Scoring", "Shooting order: LEFT -> RIGHT -> LEFT");

        // Launch all artifacts in simple order
        for (int i = 0; i < shootingOrder.length; i++) {
            LauncherSide side = shootingOrder[i];

            robot.logger.info("Scoring", "Artifact " + (i+1) + " from " + side);

            // Launch the artifact (color doesn't matter)
            scoreArtifact(side, i + 1);

            // Stop if we've run out of artifacts
            if (artifactsScored >= ARTIFACTS_TO_SCORE) {
                break;
            }
        }

        robot.logger.info("Scoring", "Scored " + artifactsScored + " artifacts");
    }

    private boolean spinUpLaunchers() {
        robot.logger.info("Launcher", "Spinning up launchers");

        // Start launchers (close shot)
        robot.launcher.spinUp(true);

        launcherSpinupTimer.reset();

        // Wait for launchers to reach minimum velocity
        while (opModeIsActive() && launcherSpinupTimer.seconds() < LAUNCHER_SPINUP_TIMEOUT) {
            double[] velocities = robot.launcher.getVelocities();
            Pose currentPose = follower.getPose();

            telemetry.addLine("=== SPINNING UP LAUNCHERS ===");
            telemetry.addLine();
            telemetry.addLine("--- ROBOT POSITION ---");
            telemetry.addData("X", "%.1f inches", currentPose.getX());
            telemetry.addData("Y", "%.1f inches", currentPose.getY());
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentPose.getHeading()));
            telemetry.addLine();
            telemetry.addLine("--- LAUNCHER STATUS ---");
            telemetry.addData("Left Velocity", "%.0f RPM", velocities[0]);
            telemetry.addData("Right Velocity", "%.0f RPM", velocities[1]);
            telemetry.addData("Spinup Time", "%.1f / %.1f sec", launcherSpinupTimer.seconds(), LAUNCHER_SPINUP_TIMEOUT);
            telemetry.addData("Ready", robot.launcher.isReady() ? "YES" : "NO");
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

    private void scoreArtifact(LauncherSide side, int artifactNum) {
        // Remove artifact from slot (we don't care about color)
        ArtifactColor removedColor = robot.artifactManager.removeNext(side);
        Pose currentPose = follower.getPose();

        telemetry.addLine("=== SCORING ARTIFACT ===");
        telemetry.addLine();
        telemetry.addLine("--- ROBOT POSITION ---");
        telemetry.addData("X", "%.1f inches", currentPose.getX());
        telemetry.addData("Y", "%.1f inches", currentPose.getY());
        telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(currentPose.getHeading()));
        telemetry.addLine();
        telemetry.addLine("--- ARTIFACT INFO ---");
        telemetry.addData("Artifact", "%d / %d", artifactNum, ARTIFACTS_TO_SCORE);
        telemetry.addData("Color", removedColor != null ? removedColor : "EMPTY");
        telemetry.addData("Feeder", side);
        telemetry.update();

        // Feed artifact (blocking)
        robot.launcher.feed(side, this::opModeIsActive);

        artifactsScored++;

        robot.logger.info("Scoring", "Artifact " + artifactNum + " scored from " + side);

        // Small delay between shots
        sleep(200);
    }
}
