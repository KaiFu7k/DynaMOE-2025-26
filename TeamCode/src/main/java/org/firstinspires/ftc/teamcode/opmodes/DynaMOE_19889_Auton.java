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
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


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
 * Uses Ivy command framework for Pedro Pathing.
 *
 * Goal-side: Scores 12 artifacts (3 preloads + 3 spikes x 3 each), parks at final launch
 * Perimeter-side: Stub — scores preloads, parks
 */
@Autonomous(name = "DynaMOE 19889 Auto", group = "Autonomous")
public class DynaMOE_19889_Auton extends LinearOpMode {


    // ==================== CONSTANTS ====================

    private static final long SPINUP_DELAY_MS = 800;
    private static final long LAUNCHER_SPINUP_TIMEOUT_MS = 1500;


    // ==================== HARDWARE & SUBSYSTEMS ====================

    private RobotHardware robot;
    private Follower follower;


    // ==================== CONFIGURATION ====================

    private StartPosition startPosition = StartPosition.BLUE_GOAL_SIDE;
    private Alliance alliance = Alliance.BLUE;
    private boolean configurationConfirmed = false;


    // ==================== ALLIANCE POSES (set in initAlliancePoses) ====================

    private Pose launchPose, launchPoseTop, finalLaunchPose;
    private Pose spikeTopApproach, spikeTopIntake;
    private Pose spikeMiddleApproach, spikeMiddleIntake, spikeMiddleRetreat;
    private Pose spikeBottomApproach, spikeBottomIntake;
    private Pose perimLaunchPose, perimParkPose;


    // ==================== PRE-BUILT PATHS ====================

    private PathChain startToLaunch;
    private PathChain launchToTopApproach, topApproachToTopIntake, topIntakeToLaunch;
    private PathChain launchToMiddleApproach, middleApproachToMiddleIntake, middleIntakeToMiddleRetreat, middleRetreatToLaunch;
    private PathChain launchToBottomApproach, bottomApproachToBottomIntake, bottomIntakeToFinalLaunch;
    // Perimeter
    private PathChain perimStartToLaunch, perimLaunchToPark;


    // ==================== MAIN ====================


    @Override
    public void runOpMode() {
        robot = new RobotHardware(telemetry);
        follower = Constants.createFollower(hardwareMap);

        configureAutonomous();

        // Skip drivetrain init — Pedro Pathing controls drive motors during auton
        robot.init(hardwareMap, null, alliance, true);

        Scheduler.reset();

        Pose startPose = FieldPositions.getStartPose(startPosition);
        follower.setStartingPose(startPose);

        initAlliancePoses();
        generatePaths();

        telemetry.addLine("=== READY TO START ===");
        telemetry.addData("Position", startPosition);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        follower.update();

        // Continuous background updates
        schedule(infinite(() -> follower.update()));
        schedule(infinite(() -> robot.updateSubsystems()));
        schedule(infinite(() -> RobotState.saveAutonEndPose(follower.getPose(), alliance)));

        if (FieldPositions.isGoalSide(startPosition)) {
            scheduleGoalSideAuto();
        } else {
            schedulePerimeterSideAuto();
        }

        while (opModeIsActive()) {
            Scheduler.execute();
        }
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
//            sleep(50);
        }
    }


    private void displayConfiguration() {
        telemetry.addLine("=== DynaMOE 19889 AUTON CONFIG ===");
        telemetry.addData("Position", startPosition);
        telemetry.addData("Confirmed", configurationConfirmed);
        telemetry.update();
    }


    // ==================== POSE & PATH SETUP ====================


    private void initAlliancePoses() {
        boolean blue = (alliance == Alliance.BLUE);

        double launchX = blue ? 48 : 96;
        double launchXTop = blue ? 52 : 92;
        double spikeX = blue ? 14 : 129;
        double spikeXTop = blue ? 18 : 126;
        double spikeXBack = blue ? 33 : 111;
        double hdg = blue ? Math.toRadians(180) : Math.toRadians(0);
        double launchHdg = blue ? Math.toRadians(135) : Math.toRadians(45);
        double finalX = blue ? 60 : 84;
        double finalHdg = blue ? Math.toRadians(149) : Math.toRadians(31);

        launchPose = new Pose(launchX, 96, launchHdg);
        launchPoseTop = new Pose(launchXTop, 84, hdg);
        finalLaunchPose = new Pose(finalX, 108, finalHdg);

        spikeTopApproach = new Pose(launchXTop, 83.5, hdg);
        spikeTopIntake = new Pose(spikeXTop, 83.5, hdg);

        spikeMiddleApproach = new Pose(launchX, 58.5, hdg);
        spikeMiddleIntake = new Pose(spikeX, 58.5, hdg);
        spikeMiddleRetreat = new Pose(spikeXBack, 58.5, hdg);

        spikeBottomApproach = new Pose(launchX, 35.5, hdg);
        spikeBottomIntake = new Pose(spikeX, 35.5, hdg);

        // Perimeter
        double perimX = blue ? 48 : 96;
        double perimHdg = blue ? Math.toRadians(110) : Math.toRadians(70);
        perimLaunchPose = new Pose(perimX, 15, perimHdg);
        perimParkPose = new Pose(85, 35, hdg);
    }


    private void generatePaths() {
        Pose startPose = FieldPositions.getStartPose(startPosition);

        // Goal-side paths
        startToLaunch = buildPath(startPose, launchPose);

        launchToTopApproach = buildPath(launchPose, spikeTopApproach);
        topApproachToTopIntake = buildPath(spikeTopApproach, spikeTopIntake);
        topIntakeToLaunch = buildPath(spikeTopIntake, launchPose);

        launchToMiddleApproach = buildPath(launchPose, spikeMiddleApproach);
        middleApproachToMiddleIntake = buildPath(spikeMiddleApproach, spikeMiddleIntake);
        middleIntakeToMiddleRetreat = buildPath(spikeMiddleIntake, spikeMiddleRetreat);
        middleRetreatToLaunch = buildPath(spikeMiddleRetreat, launchPose);

        launchToBottomApproach = buildPath(launchPose, spikeBottomApproach);
        bottomApproachToBottomIntake = buildPath(spikeBottomApproach, spikeBottomIntake);
        bottomIntakeToFinalLaunch = buildPath(spikeBottomIntake, finalLaunchPose);

        // Perimeter paths
        perimStartToLaunch = buildPath(startPose, perimLaunchPose);
        perimLaunchToPark = buildPath(perimLaunchPose, perimParkPose);
    }


    private PathChain buildPath(Pose from, Pose to) {
        return follower.pathBuilder()
                .addPath(new BezierLine(from, to))
                .setLinearHeadingInterpolation(from.getHeading(), to.getHeading())
                .build();
    }


    // ==================== GOAL-SIDE AUTO ====================


    private void scheduleGoalSideAuto() {
        double launchVel = getVelocityForPose(launchPose);
        double finalVel = getVelocityForPose(finalLaunchPose);

        schedule(sequential(
                // === SCORE PRELOADS ===
                parallel(
                        follow(follower, startToLaunch),
                        spinUpLauncher(launchVel)
                ),
                waitUntilLauncherReady(),
                setIntake(true),
                shootAll(),
                stopLauncher(),

                // === TOP SPIKE (Y=84) ===
                follow(follower, launchToTopApproach),
                setIntake(true),
                addSimulatedArtifacts(),
                follow(follower, topApproachToTopIntake),
                setIntake(false),
                parallel(
                        follow(follower, topIntakeToLaunch),
                        sequential(
                                waitMs(SPINUP_DELAY_MS-750),
                                spinUpLauncher(launchVel)
                        )
                ),
                waitUntilLauncherReady(),
                setIntake(true),
                shootAll(),
                stopLauncher(),

                // === MIDDLE SPIKE (Y=58) — retreat to avoid obstruction ===
                follow(follower, launchToMiddleApproach),
                setIntake(true),
                addSimulatedArtifacts(),
                follow(follower, middleApproachToMiddleIntake),
                follow(follower, middleIntakeToMiddleRetreat),
                setIntake(false),
                parallel(
                        follow(follower, middleRetreatToLaunch),
                        sequential(
                                waitMs(SPINUP_DELAY_MS),
                                spinUpLauncher(launchVel)
                        )
                ),
                waitUntilLauncherReady(),
                setIntake(true),
                shootAll(),
                stopLauncher(),

                // === BOTTOM SPIKE (Y=36) — goes to final launch ===
                follow(follower, launchToBottomApproach),
                setIntake(true),
                addSimulatedArtifacts(),
                follow(follower, bottomApproachToBottomIntake),
                setIntake(false),
                parallel(
                        follow(follower, bottomIntakeToFinalLaunch),
                        sequential(
                                waitMs(SPINUP_DELAY_MS),
                                spinUpLauncher(finalVel)
                        )
                ),
                waitUntilLauncherReady(),
                setIntake(true),
                shootAll(),

                // === PARK (stay at final launch position) ===
                stopLauncher(),
                setIntake(false)
        ));
    }


    // ==================== PERIMETER-SIDE AUTO (STUB) ====================


    private void schedulePerimeterSideAuto() {
        double perimVel = getVelocityForPose(perimLaunchPose);

        schedule(sequential(
                waitMs(24000),
                parallel(
                        follow(follower, perimStartToLaunch),
                        spinUpLauncher(perimVel)
                ),
                waitUntilLauncherReady(),
                setIntake(true),
                shootAll(),
                stopLauncher(),
                follow(follower, perimLaunchToPark),
                setIntake(false)
        ));
    }


    // ==================== COMMAND BUILDERS ====================


    /** Spin up launcher to a specific velocity (left = vel, right = vel - 50). */
    private Command spinUpLauncher(double velocity) {
        return instant(() -> robot.launcher.setTargetVelocity(velocity, velocity - 50));
    }

    /** Wait until launcher reports ready, with timeout. */
    private Command waitUntilLauncherReady() {
        return race(
                waitUntil(() -> robot.launcher.isReady()),
                waitMs(LAUNCHER_SPINUP_TIMEOUT_MS)
        );
    }

    /** Set intake on or off. */
    private Command setIntake(boolean on) {
        return instant(() -> {
            if (on) robot.intake.intake();
            else robot.intake.stop();
        });
    }

    /** Feed one side for 900ms, then stop feeders. */
    private Command shootSide(LauncherSide side) {
        return sequential(
                instant(() -> robot.launcher.startFeed(side)),
                waitMs(900),
                instant(() -> robot.launcher.stopFeeders())
        );
    }

    /** Fire L, R, L, R — 4 feeds guarantees all 3 artifacts launch. */
    private Command shootAll() {
        return sequential(
                shootSide(LauncherSide.LEFT),
                shootSide(LauncherSide.RIGHT),
                shootSide(LauncherSide.LEFT),
                shootSide(LauncherSide.RIGHT)
        );
    }

    /** Stop launcher motors and feeders. */
    private Command stopLauncher() {
        return instant(() -> robot.launcher.stop());
    }

    /** Add 3 simulated artifacts for tracking. */
    private Command addSimulatedArtifacts() {
        return instant(() -> {
            robot.artifactManager.addArtifact(LauncherSide.LEFT, ArtifactColor.PURPLE);
            robot.artifactManager.addArtifact(LauncherSide.RIGHT, ArtifactColor.PURPLE);
            robot.artifactManager.addArtifact(LauncherSide.LEFT, ArtifactColor.PURPLE);
        });
    }


    // ==================== UTILITY ====================


    /** Get LERP table velocity for a given launch position. */
    private double getVelocityForPose(Pose launchPose) {
        double dist = FieldPositions.getDistanceToGoal(launchPose, alliance);
        return LauncherAssist.calculateVelocity(dist);
    }
}
