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


    // ==================== ALLIANCE COORDINATES ====================


    private double launchX, launchXtop, spikeX, spikeXtop, spikeXBack;
    private double heading, launchHeading;


    // ==================== MAIN AUTONOMOUS ====================


    @Override
    public void runOpMode() {
        robot = new RobotHardware(telemetry);
        follower = Constants.createFollower(hardwareMap);

        configureAutonomous();

        // Skip drivetrain init — Pedro Pathing controls drive motors during auton
        robot.init(hardwareMap, null, alliance, true);


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


    private void initAllianceCoords() {
        launchX = (alliance == Alliance.BLUE) ? 48 : 96;
        launchXtop = (alliance == Alliance.BLUE) ? 52 : 92;
        spikeX = (alliance == Alliance.BLUE) ? 15 : 129;
        spikeXtop = (alliance == Alliance.BLUE) ? 18 : 126;
        spikeXBack = (alliance == Alliance.BLUE) ? 33 : 111;
        heading = (alliance == Alliance.BLUE) ? Math.toRadians(180) : Math.toRadians(0);
        launchHeading = (alliance == Alliance.BLUE) ? Math.toRadians(135) : Math.toRadians(45);
    }


    // ==================== DYNAMIC PATH BUILDERS ====================
    // Each method builds a path from the robot's ACTUAL current position.
    // Uses BezierLine (straight lines) for reliable, predictable movement.
    // Spike-bound paths chain intermediate + spike segments so the robot doesn't stop between them.


    /** Path A: Start to Launch — straight line */
    private PathChain buildStartToLaunch() {
        Pose current = follower.getPose();
        Pose launchPose = new Pose(launchX, 96, launchHeading);
        return follower.pathBuilder()
                .addPath(new BezierLine(current, launchPose))
                .setLinearHeadingInterpolation(current.getHeading(), launchHeading)
                .build();
    }

    /** Return to launch position — straight line from current position */
    private PathChain buildReturnToLaunch() {
        Pose current = follower.getPose();
        Pose launchPose = new Pose(launchX, 96, launchHeading);
        return follower.pathBuilder()
                .addPath(new BezierLine(current, launchPose))
                .setLinearHeadingInterpolation(current.getHeading(), launchHeading)
                .build();
    }

    /** Path G: Bottom Spike Back to Final Launch position
     *  Final launch at (60, 108) / (84, 108) to avoid standing on launch zone lines
     *  Heading adjusted: Blue ~149°, Red ~31° */
    private PathChain buildBottomSpikeToFinalLaunch() {
        Pose current = follower.getPose();
        double finalLaunchX = (alliance == Alliance.BLUE) ? 60 : 84;
        double finalLaunchHeading = (alliance == Alliance.BLUE) ? Math.toRadians(149) : Math.toRadians(31);
        Pose finalLaunchPose = new Pose(finalLaunchX, 108, finalLaunchHeading);
        return follower.pathBuilder()
                .addPath(new BezierLine(current, finalLaunchPose))
                .setLinearHeadingInterpolation(current.getHeading(), finalLaunchHeading)
                .build();
    }


    /**
     * Extended GOAL-SIDE routine with curved spline paths:
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
        robot.logger.info("Autonomous", "Executing extended GOAL-SIDE routine (chained paths)");

        initAllianceCoords();

        // === SCORE PRELOADS ===
        artifactsScored = 0;
        // Spin up while moving to launch — launcher is ready by arrival
        followPathChainWithSpinup(buildStartToLaunch(), "Step 1: Moving to LAUNCH ZONE", true);
        robot.intake.intake();  // Keep intake running during launching
        launchArtifacts();
        robot.launcher.stop();

        // === TOP SPIKE RUN (Y=84) ===
        moveToPosition(new Pose(launchXtop, 84, heading), "Step 2: Moving to TOP spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(new Pose(spikeXtop, 84, heading), "Step 3: Intaking at TOP SPIKE");

        // Spin up while returning to launch — ready to fire on arrival
        // Intake stays running during travel and launching to keep artifacts seated
        followPathChainWithSpinup(buildReturnToLaunch(), "Step 4: TOP SPIKE to LAUNCH", true);
        artifactsScored = 0;
        launchArtifacts();
        robot.launcher.stop();

        // === MIDDLE SPIKE RUN (Y=58) ===
        moveToPosition(new Pose(launchX, 58, heading), "Step 5: Moving to MIDDLE spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(new Pose(spikeX, 58, heading), "Step 6: Intaking at MIDDLE SPIKE");
        moveToPosition(new Pose(spikeXBack, 58, heading), "Step 7: Backing from wall");

        // Spin up while returning to launch
        // Intake stays running during travel and launching
        followPathChainWithSpinup(buildReturnToLaunch(), "Step 8: MIDDLE SPIKE to LAUNCH", true);
        artifactsScored = 0;
        launchArtifacts();
        robot.launcher.stop();

        // === BOTTOM SPIKE RUN (Y=36) ===
        moveToPosition(new Pose(launchX, 36, heading), "Step 9: Moving to BOTTOM spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(new Pose(spikeX, 36, heading), "Step 10: Intaking at BOTTOM SPIKE");

        // No obstruction at y=36, go directly to final launch
        // Spin up with custom velocity while returning to final launch position
        // Intake stays running during travel and launching
        followPathChainWithCustomSpinup(buildBottomSpikeToFinalLaunch(),
                "Step 11: BOTTOM SPIKE to FINAL LAUNCH", 1170, 1120);
        artifactsScored = 0;
        launchArtifacts();

        // Robot stays at final launch position (60, 108) — off the launch zone lines for parking points
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


    private void followPathChain(PathChain path, String description) {
        robot.logger.info("Movement", description);
        follower.followPath(path, true);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();
            telemetry.addData("Status", description);
            telemetry.addData("Current", String.format("%.1f, %.1f", currentPose.getX(), currentPose.getY()));
            telemetry.update();
        }
    }


    /**
     * Follow a path while spinning up the launcher simultaneously.
     * Launcher starts spinning immediately, path runs in parallel.
     * After path completes, waits for launcher to be ready if it isn't already.
     */
    private void followPathChainWithSpinup(PathChain path, String description, boolean closeShot) {
        robot.logger.info("Movement", description + " (spinning up launcher)");

        // Start launcher spinning BEFORE moving
        robot.launcher.spinUp(closeShot);

        follower.followPath(path, true);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();
            telemetry.addData("Status", description);
            telemetry.addData("Current", String.format("%.1f, %.1f", currentPose.getX(), currentPose.getY()));
            telemetry.addData("Launcher", robot.launcher.isReady() ? "READY" : "Spinning...");
            telemetry.update();
        }

        // Path done — wait for launcher if not ready yet
        launcherSpinupTimer.reset();
        while (opModeIsActive() && !robot.launcher.isReady()
                && launcherSpinupTimer.seconds() < LAUNCHER_SPINUP_TIMEOUT) {
            sleep(20);
        }
    }


    /**
     * Same as above but with custom velocity for the final launch position.
     */
    private void followPathChainWithCustomSpinup(PathChain path, String description,
                                                  double velocity, double minVelocity) {
        robot.logger.info("Movement", description + " (spinning up launcher)");

        // Start launcher spinning BEFORE moving
        robot.launcher.setTargetVelocity(velocity, minVelocity);

        follower.followPath(path, true);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
            Pose currentPose = follower.getPose();
            telemetry.addData("Status", description);
            telemetry.addData("Current", String.format("%.1f, %.1f", currentPose.getX(), currentPose.getY()));
            telemetry.addData("Launcher", robot.launcher.isReady() ? "READY" : "Spinning...");
            telemetry.update();
        }

        // Path done — wait for launcher if not ready yet
        launcherSpinupTimer.reset();
        while (opModeIsActive() && !robot.launcher.isReady()
                && launcherSpinupTimer.seconds() < LAUNCHER_SPINUP_TIMEOUT) {
            sleep(20);
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


    private void scoreAllArtifactsCustom(double velocity, double minVelocity) {
        if (!spinUpLaunchersCustom(velocity, minVelocity)) return;
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
                return true;
            }
            // Don't call follower.update() here — it actively drives motors to hold position,
            // causing the robot to move/jiggle during launching
            sleep(20);
        }
        return false;
    }


    private boolean spinUpLaunchersCustom(double velocity, double minVelocity) {
        robot.launcher.setTargetVelocity(velocity, minVelocity);
        launcherSpinupTimer.reset();

        while (opModeIsActive() && launcherSpinupTimer.seconds() < LAUNCHER_SPINUP_TIMEOUT) {
            if (robot.launcher.isReady()) {
                return true;
            }
            // Don't call follower.update() here — same reason as above
            sleep(20);
        }
        return false;
    }


    private void scoreArtifact(LauncherSide side, int artifactNum) {
        robot.artifactManager.removeNext(side);
        // Intake stays running throughout — keeps artifacts seated against feeders
        robot.launcher.feed(side, this::opModeIsActive);
        artifactsScored++;
    }
}

