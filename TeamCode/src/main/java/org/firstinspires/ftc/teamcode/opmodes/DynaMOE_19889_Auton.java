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
import org.firstinspires.ftc.teamcode.util.RobotState;


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


        // Final status - save pose for TeleOp
        Pose finalPose = follower.getPose();
        RobotState.saveAutonEndPose(finalPose, alliance);

        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Final Pose", String.format("(%.1f, %.1f, %.1f°)",
                finalPose.getX(), finalPose.getY(), Math.toDegrees(finalPose.getHeading())));
        telemetry.addLine("Pose saved for TeleOp");
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
     * 1. Score 3 Preloads
     * 2. Intake 3 artifacts from TOP spike (Y=84)
     * 3. Score 3 artifacts
     * 4. Intake 3 artifacts from MIDDLE spike (Y=58)
     * 5. Score 3 artifacts
     * 6. Intake 3 artifacts from BOTTOM spike (Y=36)
     * 7. Score 3 artifacts
     * 8. Park
     */
    private void executeGoalSideAuto() {
        robot.logger.info("Autonomous", "Executing extended GOAL-SIDE routine");


        // Get alliance-specific coordinates (Blue uses these directly, Red mirrors X)
        double launchX = (alliance == Alliance.BLUE) ? 48 : 96;
        double launchXtop = (alliance == Alliance.BLUE) ? 52 : 92;
        double spikeX = (alliance == Alliance.BLUE) ? 15 : 129;
        double spikeXtop = (alliance == Alliance.BLUE) ? 18 : 126;
        double spikeXBack=(alliance == Alliance.BLUE) ? 33 : 111;
        double parkX = (alliance == Alliance.BLUE) ? 25 : 119;
        double heading = (alliance == Alliance.BLUE) ? Math.toRadians(180) : Math.toRadians(0);
        double launchHeading = (alliance == Alliance.BLUE) ? Math.toRadians(135) : Math.toRadians(45);


        Pose launchPose = new Pose(launchX, 96, launchHeading);


        // === SCORE PRELOADS ===
        artifactsScored = 0;
        moveToPosition(launchPose, "Step 1: Moving to LAUNCH ZONE");
        scoreAllArtifacts();
        robot.launcher.stop();  // Stop launchers before intake to prevent accidental launches


        // === TOP SPIKE RUN (Y=84) ===


        Pose topIntermediatePose = new Pose(launchXtop, 84, heading);
        Pose topSpikePose = new Pose(spikeXtop, 84, heading);


        moveToPosition(topIntermediatePose, "Step 2: Moving to TOP spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(topSpikePose, "Step 3: Intaking at TOP SPIKE");


        moveToPosition(launchPose, "Step 4: Returning to LAUNCH ZONE");
        robot.intake.stop();
        artifactsScored = 0;
        scoreAllArtifacts();
        robot.launcher.stop();  // Stop launchers before intake to prevent accidental launches


        // === MIDDLE SPIKE RUN (Y=58) ===
        Pose middleIntermediatePose = new Pose(launchX, 58, heading);
        Pose middleSpikePose = new Pose(spikeX, 58, heading);
        Pose middleSpikeBack = new Pose(spikeXBack,58, heading);



        moveToPosition(middleIntermediatePose, "Step 5: Moving to MIDDLE spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(middleSpikePose, "Step 6: Intaking at MIDDLE SPIKE");
        moveToPosition(middleSpikeBack, "Back");



        moveToPosition(launchPose, "Step 7: Returning to LAUNCH ZONE");
        robot.intake.stop();
        artifactsScored = 0;
        scoreAllArtifacts();
        robot.launcher.stop();  // Stop launchers before intake to prevent accidental launches


        // === BOTTOM SPIKE RUN (Y=36) ===
        Pose bottomIntermediatePose = new Pose(launchX, 36, heading);
        Pose bottomSpikePose = new Pose(spikeX, 36, heading);


        moveToPosition(bottomIntermediatePose, "Step 8: Moving to BOTTOM spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(bottomSpikePose, "Step 9: Intaking at BOTTOM SPIKE");


        moveToPosition(launchPose, "Step 10: Returning to LAUNCH ZONE");
        robot.intake.stop();
        artifactsScored = 0;
        scoreAllArtifacts();


        // === PARK ===
        Pose parkPose = new Pose(parkX, 77, 90);
        moveToPosition(parkPose, "Step 11: Moving to PARK position");


        robot.launcher.stop();
        robot.intake.stop();
    }


    private void addSimulatedArtifacts() {
        robot.artifactManager.addArtifact(LauncherSide.LEFT, ArtifactColor.PURPLE);
        robot.artifactManager.addArtifact(LauncherSide.RIGHT, ArtifactColor.PURPLE);
        robot.artifactManager.addArtifact(LauncherSide.LEFT, ArtifactColor.PURPLE);
    }


    private void executePerimeterSideAuto() {
        robot.logger.info("Autonomous", "Executing PERIMETER-SIDE routine (far shot)");


        // Reset counter for this run
        artifactsScored = 0;

        sleep(2500);
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
        robot.intake.intake();

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
        robot.intake.intake();
        robot.launcher.feed(side, this::opModeIsActive);
        artifactsScored++;
        sleep(200);
    }
}

