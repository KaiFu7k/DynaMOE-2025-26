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
 */
@Autonomous(name = "DynaMOE 19889 Auto", group = "Autonomous")
public class DynaMOE_19889_Auton extends LinearOpMode {

    // ==================== CONSTANTS ====================

    private static final double LAUNCHER_SPINUP_TIMEOUT = 3.0;
    private static final int ARTIFACTS_TO_SCORE = 3;

    // ==================== HARDWARE & SUBSYSTEMS ====================

    private RobotHardware robot;
    private Follower follower;

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
        robot = new RobotHardware(telemetry);
        robot.init(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        configureAutonomous();

        Pose startPose = FieldPositions.getStartPose(startPosition);
        follower.setStartingPose(startPose);

        telemetry.addLine("=== READY TO START ===");
        telemetry.addData("Position", startPosition);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        follower.update();

        if (FieldPositions.isGoalSide(startPosition)) {
            executeGoalSideAuto();
        } else {
            executePerimeterSideAuto();
        }

        // Final status
        Pose finalPose = follower.getPose();
        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Final Pose", String.format("(%.1f, %.1f, %.1f°)", 
            finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading())));
        telemetry.update();
    }

    // ==================== CONFIGURATION ====================

    private void configureAutonomous() {
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_up) startPosition = StartPosition.BLUE_GOAL_SIDE;
            else if (gamepad1.dpad_down) startPosition = StartPosition.BLUE_PERIMETER_SIDE;
            else if (gamepad1.dpad_left) startPosition = StartPosition.RED_GOAL_SIDE;
            else if (gamepad1.dpad_right) startPosition = StartPosition.RED_PERIMETER_SIDE;

            if (gamepad1.a) configurationConfirmed = true;
            alliance = FieldPositions.getAlliance(startPosition);
            displayConfiguration();
            sleep(50);
        }
    }

    private void displayConfiguration() {
        telemetry.addLine("=== DynaMOE 19889 AUTON CONFIG ===");
        telemetry.addData("Position", startPosition);
        telemetry.addData("Confirmed", configurationConfirmed);
        telemetry.update();
    }

    // ==================== AUTONOMOUS SEQUENCES ====================

    /**
     * Extended GOAL-SIDE routine:
     * 1. Score Preloads
     * 2. Intake 3 artifacts from Spike
     * 3. Score 3 collected artifacts
     * 4. Park
     */
    private void executeGoalSideAuto() {
        robot.logger.info("Autonomous", "Executing extended GOAL-SIDE routine");

        // Reset counter for this run
        artifactsScored = 0;

        // Step 1: Navigate to LAUNCH ZONE and Score Preloads
        Pose launchPose = FieldPositions.getLaunchPose(alliance);
        moveToPosition(launchPose, "Step 1: Moving to LAUNCH ZONE");
        scoreAllArtifacts();

        // Step 2: Move to intermediate position to prepare for intake run
        Pose leavePose = FieldPositions.getLeavePose(alliance); // (48, 58, 180)
        moveToPosition(leavePose, "Step 2: Positioning for intake run");

        // Step 3: Intake 3 artifacts at middle spike
        robot.logger.info("Autonomous", "Starting intake run");
        robot.intake.intake();
        
        // Manually update manager (simulating intake)
        robot.artifactManager.addArtifact(LauncherSide.LEFT, ArtifactColor.PURPLE);
        robot.artifactManager.addArtifact(LauncherSide.RIGHT, ArtifactColor.PURPLE);
        robot.artifactManager.addArtifact(LauncherSide.LEFT, ArtifactColor.PURPLE);

        Pose spikePose = FieldPositions.getSpikeMiddlePose(alliance); // (15, 58, 180)
        moveToPosition(spikePose, "Step 3: Intaking artifacts at SPIKE");
        
        // Move back to intermediate position before returning to launch zone
        moveToPosition(leavePose, "Step 3.5: Moving back from SPIKE");

        // Step 4: Return to LAUNCH ZONE
        moveToPosition(launchPose, "Step 4: Returning to LAUNCH ZONE");
        robot.intake.stop();

        // Step 5: Score the 3 collected artifacts
        artifactsScored = 0; // Reset counter for second batch
        scoreAllArtifacts();

        // Step 6: Move to final parking position
        Pose parkPose = FieldPositions.getParkPose(alliance); // (48, 36, 180)
        moveToPosition(parkPose, "Step 6: Moving to final PARK position");

        // Clean up
        robot.launcher.stop();
        robot.intake.stop();
    }

    private void executePerimeterSideAuto() {
        robot.logger.info("Autonomous", "Executing PERIMETER-SIDE routine (far shot)");

        // Reset counter for this run
        artifactsScored = 0;

        // Step 1: Move to perimeter launch position
        Pose launchPose = FieldPositions.getPerimeterLaunchPose(alliance);
        moveToPosition(launchPose, "Step 1: Moving to LAUNCH position");

        // Step 2: Score preloads with far shot
        scoreAllArtifactsFar();

        // Clean up
        robot.launcher.stop();
        robot.intake.stop();
    }

    // ==================== MOVEMENT ====================

    private void moveToPosition(Pose targetPose, String description) {
        robot.logger.info("Movement", description);
        Pose currentPose = follower.getPose();
        
        PathChain path = follower.pathBuilder()
            .addPath(new BezierLine(new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading()), targetPose))
            .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
            .build();

        follower.followPath(path, true);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
            updatePositionTelemetry(description, targetPose);
        }
    }

    private void updatePositionTelemetry(String status, Pose targetPose) {
        Pose currentPose = follower.getPose();
        telemetry.addData("Status", status);
        telemetry.addData("Current", String.format("%.1f, %.1f", currentPose.getX(), currentPose.getY()));
        telemetry.addData("Target", String.format("%.1f, %.1f", targetPose.getX(), targetPose.getY()));
        telemetry.update();
    }

    // ==================== LAUNCHER CONTROL ====================

    private void scoreAllArtifacts() {
        if (!spinUpLaunchers(true)) return;  // close shot
        launchArtifacts();
    }

    private void scoreAllArtifactsFar() {
        if (!spinUpLaunchers(false)) return;  // far shot
        launchArtifacts();
    }

    private void launchArtifacts() {
        LauncherSide[] shootingOrder = { LauncherSide.LEFT, LauncherSide.RIGHT, LauncherSide.LEFT };

        for (int i = 0; i < shootingOrder.length; i++) {
            if (artifactsScored >= ARTIFACTS_TO_SCORE) break;
            scoreArtifact(shootingOrder[i], i + 1);
        }
    }

    private boolean spinUpLaunchers(boolean closeShot) {
        robot.launcher.spinUp(closeShot);
        launcherSpinupTimer.reset();

        while (opModeIsActive() && launcherSpinupTimer.seconds() < LAUNCHER_SPINUP_TIMEOUT) {
            if (robot.launcher.isReady()) {
                robot.intake.intake();
                return true;
            }
            follower.update();
            sleep(20);
        }
        return false;
    }

    private void scoreArtifact(LauncherSide side, int artifactNum) {
        robot.artifactManager.removeNext(side);
        robot.launcher.feed(side, this::opModeIsActive);
        artifactsScored++;
        sleep(200);
    }
}
