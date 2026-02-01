package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.RobotHardware;
import org.firstinspires.ftc.teamcode.util.FieldPositions;
import org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;

/**
 * LAUNCHER VELOCITY TUNING OPMODE
 *
 * PURPOSE:
 * Use this OpMode to find the correct launcher velocity (RPM) for each distance.
 * The data collected here should be used to update the VELOCITY_TABLE in LauncherAssist.java
 *
 * HOW TO USE:
 * 1. Select alliance (determines which goal to measure distance to)
 * 2. Select starting position (sets initial robot pose for odometry)
 * 3. Drive robot to a known distance from the goal (use tape measure or field markings)
 * 4. Use D-pad to adjust launcher velocity until shots consistently land in goal
 * 5. Record the distance and velocity shown on telemetry
 * 6. Repeat at different distances (recommended: 24", 36", 48", 60", 72", 84")
 * 7. Update VELOCITY_TABLE in LauncherAssist.java with your recorded values
 *
 * CONTROLS:
 * === PRE-INIT (Configuration) ===
 * X: Select Blue Alliance
 * B: Select Red Alliance
 * D-Pad Up: Goal Side start
 * D-Pad Down: Perimeter Side start
 *
 * === DURING MATCH ===
 * Left Stick: Drive (robot-centric)
 * Right Stick X: Rotate
 *
 * D-Pad Up: Increase velocity by 50 RPM
 * D-Pad Down: Decrease velocity by 50 RPM
 * D-Pad Right: Increase velocity by 10 RPM (fine tune)
 * D-Pad Left: Decrease velocity by 10 RPM (fine tune)
 *
 * A: Spin up launcher
 * B: Stop launcher
 * X: Feed LEFT artifact
 * Y: Feed RIGHT artifact
 *
 * Right Trigger: Intake
 * Left Trigger: Outtake
 *
 * Left Bumper: Toggle auto-align ON/OFF
 * Right Bumper: Save current reading to log
 */
@TeleOp(name = "Launcher Velocity Tuner", group = "Tuning")
public class LauncherVelocityTuner extends LinearOpMode {

    // Hardware
    private RobotHardware robot;
    private Follower follower;

    // Configuration
    private FieldPositions.Alliance alliance = FieldPositions.Alliance.BLUE;
    private FieldPositions.StartPosition startPos = FieldPositions.StartPosition.BLUE_GOAL_SIDE;

    // Tuning state
    private double currentVelocity = 1200;  // Starting velocity
    private static final double VELOCITY_MIN = 500;
    private static final double VELOCITY_MAX = 2000;
    private static final double COARSE_INCREMENT = 50;
    private static final double FINE_INCREMENT = 10;

    // Auto-align
    private boolean autoAlignActive = false;

    // Debounce timers
    private ElapsedTime dpadTimer = new ElapsedTime();
    private ElapsedTime bumperTimer = new ElapsedTime();
    private static final double DEBOUNCE_TIME = 0.2;  // 200ms

    // Data logging
    private int savedReadings = 0;
    private StringBuilder tuningLog = new StringBuilder();

    @Override
    public void runOpMode() {
        // === CONFIGURATION PHASE ===
        telemetry.addLine("=== LAUNCHER VELOCITY TUNER ===");
        telemetry.addLine("Initializing Follower...");
        telemetry.update();

        try {
            follower = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            telemetry.addData("Follower Error", e.getMessage());
            telemetry.addLine("Cannot run tuner without odometry!");
            telemetry.update();
            return;
        }

        // Configuration loop
        while (!isStarted() && !isStopRequested()) {
            // Alliance Selection
            if (gamepad1.x) alliance = FieldPositions.Alliance.BLUE;
            if (gamepad1.b) alliance = FieldPositions.Alliance.RED;

            // Start Position Selection
            if (gamepad1.dpad_up) {
                startPos = (alliance == FieldPositions.Alliance.BLUE) ?
                        FieldPositions.StartPosition.BLUE_GOAL_SIDE : FieldPositions.StartPosition.RED_GOAL_SIDE;
            }
            if (gamepad1.dpad_down) {
                startPos = (alliance == FieldPositions.Alliance.BLUE) ?
                        FieldPositions.StartPosition.BLUE_PERIMETER_SIDE : FieldPositions.StartPosition.RED_PERIMETER_SIDE;
            }

            telemetry.addLine("=== LAUNCHER VELOCITY TUNER ===");
            telemetry.addLine();
            telemetry.addData("Alliance", alliance == FieldPositions.Alliance.BLUE ? "BLUE" : "RED");
            telemetry.addData("Start Position", startPos);
            telemetry.addLine();
            telemetry.addLine("X: Blue | B: Red");
            telemetry.addLine("D-Pad Up: Goal Side | D-Pad Down: Perimeter");
            telemetry.addLine();
            telemetry.addLine("Press START to begin tuning");
            telemetry.update();

            sleep(50);
        }

        if (isStopRequested()) return;

        // Initialize hardware
        robot = new RobotHardware(telemetry);
        robot.init(hardwareMap, follower, alliance);

        // Set starting pose
        follower.setStartingPose(FieldPositions.getStartPose(startPos));

        // Initialize log header
        tuningLog.append("=== VELOCITY TUNING LOG ===\n");
        tuningLog.append("Alliance: ").append(alliance).append("\n");
        tuningLog.append("Format: Distance (in) -> Velocity (RPM)\n");
        tuningLog.append("----------------------------\n");

        telemetry.addLine("Ready to tune!");
        telemetry.update();

        waitForStart();
        dpadTimer.reset();
        bumperTimer.reset();

        // === MAIN TUNING LOOP ===
        while (opModeIsActive()) {
            // Update odometry
            follower.update();

            // Update launcher assist for distance calculation
            if (robot.launcherAssist != null) {
                robot.launcherAssist.update();
            }

            // Handle all controls
            handleDriveControls();
            handleVelocityControls();
            handleLauncherControls();
            handleIntakeControls();
            handleAutoAlignToggle();
            handleSaveReading();

            // Update subsystems
            robot.updateSubsystems();

            // Display telemetry
            updateTelemetry();
        }

        // Print final log
        telemetry.addLine("=== TUNING COMPLETE ===");
        telemetry.addLine(tuningLog.toString());
        telemetry.update();

        robot.stopAllSubsystems();
    }

    private void handleDriveControls() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx;

        // Use auto-align rotation if enabled
        if (autoAlignActive && robot.launcherAssist != null) {
            rx = robot.launcherAssist.getRotationPower();
        } else {
            rx = gamepad1.right_stick_x;
        }

        // Robot-centric drive (simpler for tuning)
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double leftFrontPower = (y + x + rx) / denominator;
        double leftBackPower = (y - x + rx) / denominator;
        double rightFrontPower = (y - x - rx) / denominator;
        double rightBackPower = (y + x - rx) / denominator;

        robot.drivetrain.setPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    private void handleVelocityControls() {
        if (dpadTimer.seconds() < DEBOUNCE_TIME) return;

        boolean changed = false;

        // Coarse adjustment (50 RPM)
        if (gamepad1.dpad_up) {
            currentVelocity = Math.min(currentVelocity + COARSE_INCREMENT, VELOCITY_MAX);
            changed = true;
        } else if (gamepad1.dpad_down) {
            currentVelocity = Math.max(currentVelocity - COARSE_INCREMENT, VELOCITY_MIN);
            changed = true;
        }
        // Fine adjustment (10 RPM)
        else if (gamepad1.dpad_right) {
            currentVelocity = Math.min(currentVelocity + FINE_INCREMENT, VELOCITY_MAX);
            changed = true;
        } else if (gamepad1.dpad_left) {
            currentVelocity = Math.max(currentVelocity - FINE_INCREMENT, VELOCITY_MIN);
            changed = true;
        }

        if (changed) {
            dpadTimer.reset();
            // Update launcher if it's already spinning
            if (robot.launcher.isInitialized()) {
                double[] velocities = robot.launcher.getVelocities();
                if (velocities[0] > 100 || velocities[1] > 100) {
                    // Launcher is spinning, update velocity
                    robot.launcher.setTargetVelocity(currentVelocity, currentVelocity - 50);
                }
            }
        }
    }

    private void handleLauncherControls() {
        // A: Spin up launcher at current velocity
        if (gamepad1.a) {
            robot.launcher.setTargetVelocity(currentVelocity, currentVelocity - 50);
        }

        // B: Stop launcher
        if (gamepad1.b) {
            robot.launcher.stop();
        }

        // X: Feed LEFT
        if (gamepad1.x && robot.launcher.isReady()) {
            robot.launcher.startFeed(LauncherSide.LEFT);
        }

        // Y: Feed RIGHT
        if (gamepad1.y && robot.launcher.isReady()) {
            robot.launcher.startFeed(LauncherSide.RIGHT);
        }
    }

    private void handleIntakeControls() {
        if (gamepad1.right_trigger > 0.5) {
            robot.intake.intake();
        } else if (gamepad1.left_trigger > 0.5) {
            robot.intake.outtake();
        } else {
            robot.intake.stop();
        }
    }

    private void handleAutoAlignToggle() {
        if (gamepad1.left_bumper && bumperTimer.seconds() > DEBOUNCE_TIME) {
            autoAlignActive = !autoAlignActive;
            if (robot.launcherAssist != null) {
                robot.launcherAssist.resetPID();
            }
            bumperTimer.reset();
        }
    }

    private void handleSaveReading() {
        if (gamepad1.right_bumper && bumperTimer.seconds() > DEBOUNCE_TIME) {
            if (robot.launcherAssist != null) {
                double distance = robot.launcherAssist.getDistanceToGoal();
                savedReadings++;

                String entry = String.format("%d. %.1f in -> %.0f RPM",
                        savedReadings, distance, currentVelocity);
                tuningLog.append(entry).append("\n");

                // Flash confirmation on telemetry
                telemetry.addLine(">>> SAVED: " + entry);
                telemetry.update();
                sleep(500);  // Brief pause to show confirmation
            }
            bumperTimer.reset();
        }
    }

    private void updateTelemetry() {
        telemetry.addLine("=== LAUNCHER VELOCITY TUNER ===");
        telemetry.addLine();

        // Distance and position info
        if (robot.launcherAssist != null) {
            double distance = robot.launcherAssist.getDistanceToGoal();
            telemetry.addData("DISTANCE TO GOAL", "%.1f inches", distance);
            telemetry.addData("Angle Error", "%.1f°", robot.launcherAssist.getAngleErrorDegrees());
            telemetry.addData("Aligned", robot.launcherAssist.isAligned() ? "YES ✓" : "NO");
        }

        Pose pose = follower.getPose();
        telemetry.addData("Robot Position", "(%.1f, %.1f)", pose.getX(), pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));

        telemetry.addLine();

        // Velocity tuning info
        telemetry.addLine("--- VELOCITY TUNING ---");
        telemetry.addData("TARGET VELOCITY", "%.0f RPM", currentVelocity);

        double[] actualVel = robot.launcher.getVelocities();
        telemetry.addData("Actual L/R", "%.0f / %.0f RPM", actualVel[0], actualVel[1]);
        telemetry.addData("Launcher Ready", robot.launcher.isReady() ? "YES ✓" : "NO (spinning up...)");

        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addData("Auto-Align", autoAlignActive ? "ON (LB to disable)" : "OFF (LB to enable)");
        telemetry.addLine("D-Pad U/D: ±50 RPM | L/R: ±10 RPM");
        telemetry.addLine("A: Spin up | B: Stop");
        telemetry.addLine("X: Feed LEFT | Y: Feed RIGHT");
        telemetry.addLine("RB: Save reading to log");

        telemetry.addLine();
        telemetry.addData("Saved Readings", savedReadings);

        // Show recent saves
        if (savedReadings > 0) {
            telemetry.addLine();
            telemetry.addLine("--- SAVED DATA ---");
            String[] lines = tuningLog.toString().split("\n");
            // Show last 3 saved readings
            int start = Math.max(4, lines.length - 3);  // Skip header lines
            for (int i = start; i < lines.length; i++) {
                telemetry.addLine(lines[i]);
            }
        }

        telemetry.update();
    }
}
