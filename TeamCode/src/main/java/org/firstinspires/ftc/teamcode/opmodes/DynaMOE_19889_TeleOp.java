package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;
import org.firstinspires.ftc.teamcode.util.FieldPositions;

/**
 * TeleOp OpMode for DynaMOE Team 19889
 */
@TeleOp(name = "DynaMOE 19889 TeleOp", group = "TeleOp")
public class DynaMOE_19889_TeleOp extends LinearOpMode {

    private RobotHardware robot;
    private Follower follower;

    private boolean fieldCentric = false;
    private boolean launcherActive = false;

    private static final double LAUNCHER_MIN_SPEED = 800;
    private static final double LAUNCHER_MAX_SPEED = 1600;
    private static final double LAUNCHER_SPEED_INCREMENT = 50;
    private double manualLauncherSpeed = 1200;

    private boolean autoAlignActive = false;
    private boolean autoVelocityMode = false;
    private FieldPositions.Alliance alliance = FieldPositions.Alliance.BLUE;
    private FieldPositions.StartPosition startPos = FieldPositions.StartPosition.BLUE_GOAL_SIDE;

    private double currentLFPower = 0;
    private double currentRFPower = 0;
    private double currentLBPower = 0;
    private double currentRBPower = 0;

    @Override
    public void runOpMode() {
        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Follower Error", e.getMessage());
            follower = null;
        }

        // === CONFIGURATION PHASE ===
        while (!isStarted() && !isStopRequested()) {
            // Alliance Selection
            if (gamepad1.x) alliance = FieldPositions.Alliance.BLUE;
            if (gamepad1.b) alliance = FieldPositions.Alliance.RED;

            // Start Side Selection
            if (gamepad1.dpad_up) {
                startPos = (alliance == FieldPositions.Alliance.BLUE) ? 
                    FieldPositions.StartPosition.BLUE_GOAL_SIDE : FieldPositions.StartPosition.RED_GOAL_SIDE;
            }
            if (gamepad1.dpad_down) {
                startPos = (alliance == FieldPositions.Alliance.BLUE) ? 
                    FieldPositions.StartPosition.BLUE_PERIMETER_SIDE : FieldPositions.StartPosition.RED_PERIMETER_SIDE;
            }

            telemetry.addLine("=== DynaMOE 19889 TELEOP CONFIG ===");
            telemetry.addData("Alliance", alliance == FieldPositions.Alliance.BLUE ? "BLUE" : "RED");
            telemetry.addData("Start Side", startPos);
            telemetry.addLine();
            telemetry.addLine("X: Blue | B: Red");
            telemetry.addLine("D-Pad UP: Goal Side | D-Pad DOWN: Perimeter");
            telemetry.addLine();
            telemetry.addLine("Press START to initialize hardware");
            telemetry.update();
            
            sleep(10);
        }

        if (isStopRequested()) return;

        // Initialize hardware
        robot = new RobotHardware(telemetry);
        robot.init(hardwareMap, follower, alliance);

        // SET STARTING POSE - This is the critical fix for distance and angle
        if (follower != null) {
            follower.setStartingPose(FieldPositions.getStartPose(startPos));
        }

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            robot.updateSubsystems();

            handleDriveControls();
            handleIntakeControls();
            handleAutoAlignControls();
            handleLauncherControls();

            updateTelemetry();
        }

        robot.stopAllSubsystems();
    }

    private void handleDriveControls() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx;

        if (autoAlignActive && robot.launcherAssist != null) {
            rx = robot.launcherAssist.getRotationPower();
        } else {
            rx = gamepad1.right_stick_x;
        }

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

    private void handleIntakeControls() {
        if (gamepad1.right_trigger > 0.5) robot.intake.intake();
        else if (gamepad1.left_trigger > 0.5) robot.intake.outtake();
        else robot.intake.stop();
    }

    private void handleAutoAlignControls() {
        if (robot.launcherAssist == null) return;

        if (gamepad1.left_bumper) {
            autoAlignActive = !autoAlignActive;
            robot.launcherAssist.resetPID();
            sleep(250); // debounce
        }

        if (gamepad1.right_bumper) {
            autoVelocityMode = !autoVelocityMode;
            sleep(250); // debounce
        }

        if (autoVelocityMode) {
            manualLauncherSpeed = robot.launcherAssist.getRecommendedVelocity();
            if (launcherActive) {
                robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            }
        }
    }

    private void handleLauncherControls() {
        if (gamepad1.dpad_up) manualLauncherSpeed = Math.min(manualLauncherSpeed + LAUNCHER_SPEED_INCREMENT, LAUNCHER_MAX_SPEED);
        if (gamepad1.dpad_down) manualLauncherSpeed = Math.max(manualLauncherSpeed - LAUNCHER_SPEED_INCREMENT, LAUNCHER_MIN_SPEED);

        if (gamepad1.a) {
            robot.launcher.setTargetVelocity(manualLauncherSpeed, manualLauncherSpeed - 25);
            launcherActive = true;
        }
        if (gamepad1.b) {
            robot.launcher.stop();
            launcherActive = false;
        }
        if (gamepad1.x && launcherActive && robot.launcher.isReady()) robot.launcher.startFeed(LauncherSide.LEFT);
        if (gamepad1.y && launcherActive && robot.launcher.isReady()) robot.launcher.startFeed(LauncherSide.RIGHT);
    }

    private void updateTelemetry() {
        telemetry.addLine("=== DYNAMOE 19889 TELEOP ===");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Drive Mode", fieldCentric ? "Field-Centric" : "Robot-Centric");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(follower.getHeading()));
        
        if (robot.launcherAssist != null) {
            telemetry.addData("Auto-Align", autoAlignActive ? "ACTIVE" : "Off");
            telemetry.addData("Distance", "%.1f in", robot.launcherAssist.getDistanceToGoal());
            telemetry.addData("Angle Error", "%.1f°", robot.launcherAssist.getAngleErrorDegrees());
            telemetry.addData("Target Angle", "%.1f°", robot.launcherAssist.getTargetAngleDegrees());
            telemetry.addData("Aligned", robot.launcherAssist.isAligned() ? "YES" : "NO");
        }

        telemetry.addData("Launcher Target", "%.0f RPM", manualLauncherSpeed);
        telemetry.addData("Ready", robot.launcher.isReady() ? "YES" : "NO");
        telemetry.update();
    }
}
