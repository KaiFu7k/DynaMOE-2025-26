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
import org.firstinspires.ftc.teamcode.subsystems.LauncherAssist;
import org.firstinspires.ftc.teamcode.util.FieldPositions;
import org.firstinspires.ftc.teamcode.util.FieldPositions.*;
import org.firstinspires.ftc.teamcode.util.RobotEnums.*;
import static org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;
import org.firstinspires.ftc.teamcode.util.RobotState;


/**
 * Autonomous OpMode for DynaMOE Team 19889 — DECODE 2025-26
 *
 * Goal-side: Scores 12 artifacts (3 preloads + 3 spikes x 3 each), parks at (60,108)/(84,108)
 * Perimeter-side: Scores 9 artifacts (3 preloads + 2 spikes x 3 each), parks at (36,12)/(108,12)
 *
 * Key features:
 * - BezierLine paths (straight lines) for reliable movement
 * - Launcher spins up while traveling to launch position (saves ~2s per cycle)
 * - Intake runs continuously during scoring to keep artifacts seated
 * - No follower.update() during scoring to prevent robot jiggle
 * - Pose saved for TeleOp via RobotState at end of auton
 */
@Autonomous(name = "DynaMOE 19889 Auto", group = "Autonomous")
public class DynaMOE_19889_Auton extends LinearOpMode {


    // ==================== CONSTANTS ====================


    private static final double LAUNCHER_SPINUP_TIMEOUT = 3.0;


    // ==================== HARDWARE & SUBSYSTEMS ====================


    private RobotHardware robot;
    private Follower follower;


    // ==================== CONFIGURATION ====================


    private StartPosition startPosition = StartPosition.BLUE_GOAL_SIDE;
    private Alliance alliance = Alliance.BLUE;
    private boolean configurationConfirmed = false;


    // ==================== STATE TRACKING ====================


    private ElapsedTime launcherSpinupTimer = new ElapsedTime();


    // ==================== ALLIANCE COORDINATES ====================


    private double launchX, launchXtop, spikeX, spikeXtop, spikeXBack;
    private double heading, launchHeading;
    // Perimeter-side specific
    private double perimLaunchX, perimLaunchY, perimLaunchHeading, parkX;


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
        // Perimeter-side coords
        perimLaunchX = (alliance == Alliance.BLUE) ? 48 : 96;
        perimLaunchY = 15;
        perimLaunchHeading = (alliance == Alliance.BLUE) ? Math.toRadians(110) : Math.toRadians(70);
        parkX = (alliance == Alliance.BLUE) ? 36 : 108;
    }


    // ==================== DYNAMIC PATH BUILDERS ====================
    // Each path is built from the robot's current position for accuracy.

    /** Start to goal-side launch position */
    private PathChain buildStartToLaunch() {
        Pose current = follower.getPose();
        Pose launchPose = new Pose(launchX, 96, launchHeading);
        return follower.pathBuilder()
                .addPath(new BezierLine(current, launchPose))
                .setLinearHeadingInterpolation(current.getHeading(), launchHeading)
                .build();
    }

    /** Return to goal-side launch position */
    private PathChain buildReturnToLaunch() {
        Pose current = follower.getPose();
        Pose launchPose = new Pose(launchX, 96, launchHeading);
        return follower.pathBuilder()
                .addPath(new BezierLine(current, launchPose))
                .setLinearHeadingInterpolation(current.getHeading(), launchHeading)
                .build();
    }

    /** Return to perimeter launch position */
    private PathChain buildReturnToPerimeterLaunch() {
        Pose current = follower.getPose();
        Pose launchPose = new Pose(perimLaunchX, perimLaunchY, perimLaunchHeading);
        return follower.pathBuilder()
                .addPath(new BezierLine(current, launchPose))
                .setLinearHeadingInterpolation(current.getHeading(), perimLaunchHeading)
                .build();
    }

    /** Bottom spike to final launch at (60,108)/(84,108), off launch zone lines for parking points */
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
     * GOAL-SIDE routine — 12 artifacts total:
     * Preloads -> Top spike -> Middle spike (wall retreat) -> Bottom spike -> Final launch & park
     */
    private void executeGoalSideAuto() {
        robot.logger.info("Autonomous", "Executing GOAL-SIDE routine");

        initAllianceCoords();
        Pose launchPose = new Pose(launchX, 96, launchHeading);
        double finalLaunchX = (alliance == Alliance.BLUE) ? 60 : 84;
        double finalLaunchHeading = (alliance == Alliance.BLUE) ? Math.toRadians(149) : Math.toRadians(31);
        Pose finalLaunchPose = new Pose(finalLaunchX, 108, finalLaunchHeading);

        // === SCORE PRELOADS (spin up while moving) ===
        followPathChainWithSpinup(buildStartToLaunch(), "Step 1: Moving to LAUNCH ZONE", launchPose);
        robot.intake.intake();
        launchArtifacts();
        robot.launcher.stop();

        // === TOP SPIKE (Y=84) ===
        moveToPosition(new Pose(launchXtop, 84, heading), "Step 2: Moving to TOP spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(new Pose(spikeXtop, 84, heading), "Step 3: Intaking at TOP SPIKE");
        followPathChainWithSpinup(buildReturnToLaunch(), "Step 4: TOP SPIKE to LAUNCH", launchPose);
        launchArtifacts();
        robot.launcher.stop();

        // === MIDDLE SPIKE (Y=58) — backs up to x=33/111 to avoid obstruction at (0-12, 72) ===
        moveToPosition(new Pose(launchX, 58, heading), "Step 5: Moving to MIDDLE spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(new Pose(spikeX, 58, heading), "Step 6: Intaking at MIDDLE SPIKE");
        moveToPosition(new Pose(spikeXBack, 58, heading), "Step 7: Backing from wall");
        followPathChainWithSpinup(buildReturnToLaunch(), "Step 8: MIDDLE SPIKE to LAUNCH", launchPose);
        launchArtifacts();
        robot.launcher.stop();

        // === BOTTOM SPIKE (Y=36) — no obstruction, goes directly to final launch ===
        moveToPosition(new Pose(launchX, 36, heading), "Step 9: Moving to BOTTOM spike area");
        robot.intake.intake();
        addSimulatedArtifacts();
        moveToPosition(new Pose(spikeX, 36, heading), "Step 10: Intaking at BOTTOM SPIKE");
        followPathChainWithSpinup(buildBottomSpikeToFinalLaunch(),
                "Step 11: BOTTOM SPIKE to FINAL LAUNCH", finalLaunchPose);
        launchArtifacts();

        // Park at final launch position — off launch zone lines for parking points
        robot.launcher.stop();
        robot.intake.stop();
    }


    private void addSimulatedArtifacts() {
        robot.artifactManager.addArtifact(LauncherSide.LEFT, ArtifactColor.PURPLE);
        robot.artifactManager.addArtifact(LauncherSide.RIGHT, ArtifactColor.PURPLE);
        robot.artifactManager.addArtifact(LauncherSide.LEFT, ArtifactColor.PURPLE);
    }


    /**
     * PERIMETER-SIDE routine — 9 artifacts total:
     * Preloads -> Bottom spike -> Middle spike -> Park at (36,12)/(108,12)
     * All launches use LERP table velocity based on distance to goal
     */
    private void executePerimeterSideAuto() {
        robot.logger.info("Autonomous", "Executing PERIMETER-SIDE routine");

        initAllianceCoords();

        // === SCORE PRELOADS (spin up while moving) ===
        Pose perimLaunchPose = new Pose(perimLaunchX, perimLaunchY, perimLaunchHeading);
        sleep(24000);
        followPathChainWithSpinup(
                follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), perimLaunchPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), perimLaunchHeading)
                        .build(),
                "Step 1: Moving to PERIMETER LAUNCH", perimLaunchPose);
        robot.intake.intake();
        launchArtifacts();
        robot.launcher.stop();

        // === BOTTOM SPIKE (Y=36) ===
//        moveToPosition(new Pose(perimLaunchX, 36, heading), "Step 2: Moving to BOTTOM spike area");
//        robot.intake.intake();
//        addSimulatedArtifacts();
//        moveToPosition(new Pose(spikeX, 36, heading), "Step 3: Intaking at BOTTOM SPIKE");
//        followPathChainWithSpinup(buildReturnToPerimeterLaunch(),
//                "Step 4: BOTTOM SPIKE to PERIMETER LAUNCH", perimLaunchPose);
//        launchArtifacts();
//        robot.launcher.stop();
//
//        // === MIDDLE SPIKE (Y=58) ===
//        moveToPosition(new Pose(perimLaunchX, 58, heading), "Step 5: Moving to MIDDLE spike area");
//        robot.intake.intake();
//        addSimulatedArtifacts();
//        moveToPosition(new Pose(spikeX, 58, heading), "Step 6: Intaking at MIDDLE SPIKE");
//        followPathChainWithSpinup(buildReturnToPerimeterLaunch(),
//                "Step 7: MIDDLE SPIKE to PERIMETER LAUNCH", perimLaunchPose);
//        launchArtifacts();
//        robot.launcher.stop();
//
//        // === PARK ===
          moveToPosition(new Pose(85, 35, heading), "Step 8: PARKING");
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
            RobotState.saveAutonEndPose(follower.getPose(), alliance);
            updatePositionTelemetry(description, targetPose);
        }
    }


    /** Follow path while spinning up launcher to LERP-table velocity for the destination. */
    private void followPathChainWithSpinup(PathChain path, String description, Pose destPose) {
        robot.logger.info("Movement", description + " (spinning up launcher)");

        // Calculate velocity from LERP table based on destination distance to goal
        double velocity = getVelocityForPose(destPose);
        robot.launcher.setTargetVelocity(velocity, velocity - 50);

        follower.followPath(path, true);
        while (follower.isBusy() && opModeIsActive()) {
            follower.update();
            RobotState.saveAutonEndPose(follower.getPose(), alliance);
            Pose currentPose = follower.getPose();
            telemetry.addData("Status", description);
            telemetry.addData("Current", String.format("%.1f, %.1f", currentPose.getX(), currentPose.getY()));
            telemetry.addData("Launcher Vel", "%.0f RPM", velocity);
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

    /** Get LERP table velocity for a given launch position using LauncherAssist's table */
    private double getVelocityForPose(Pose launchPose) {
        double dist = FieldPositions.getDistanceToGoal(launchPose, alliance);
        return LauncherAssist.calculateVelocity(dist);
    }

    /** Fire L, R, L, R — 4 feeds guarantees all 3 artifacts launch regardless of which side they're on.
     *  Uses the center heading (splits the difference between L and R offsets). */
    private void launchArtifacts() {
        LauncherSide[] order = { LauncherSide.LEFT, LauncherSide.RIGHT, LauncherSide.LEFT, LauncherSide.RIGHT };
        for (LauncherSide side : order) {
            robot.artifactManager.removeNext(side);
            robot.launcher.feed(side, this::opModeIsActive);
        }
    }
}

