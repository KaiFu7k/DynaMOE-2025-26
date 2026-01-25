package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;

/**
 * Launcher subsystem for DECODE game
 * Controls launcher motors, feeder servos, and diverter servo
 * Used by both Autonomous and TeleOp
 */
public class Launcher {

    // Constants
    public static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1150;  // RPM for close shots
    public static final double LAUNCHER_CLOSE_MIN_VELOCITY = 1100;     // Minimum acceptable RPM
    public static final double LAUNCHER_FAR_TARGET_VELOCITY = 1520;    // RPM for far shots
    public static final double LAUNCHER_FAR_MIN_VELOCITY = 1450;       // Minimum for far shots (wider margin for reliability)

    private static final double FEED_TIME_SECONDS = 0.80;  // Time to feed one artifact
    private static final double STOP_SPEED = 0.0;
    private static final double FULL_SPEED = 1.0;

    // Diverter servo positions
    private static final double LEFT_POSITION = 0.2962;
    private static final double RIGHT_POSITION = 0;

    // Hardware
    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private Servo diverter;

    // Telemetry
    private Telemetry telemetry;

    // State tracking
    private boolean isInitialized = false;
    private double targetVelocity = LAUNCHER_CLOSE_TARGET_VELOCITY;
    private double minVelocity = LAUNCHER_CLOSE_MIN_VELOCITY;
    private ElapsedTime leftFeederTimer = new ElapsedTime();
    private ElapsedTime rightFeederTimer = new ElapsedTime();
    private boolean leftFeeding = false;
    private boolean rightFeeding = false;

    /**
     * Constructor
     * @param telemetry Telemetry object for debug output
     */
    public Launcher(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Initialize hardware from HardwareMap
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize launchers
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");

        // Initialize feeders
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        // Initialize diverter
        diverter = hardwareMap.get(Servo.class, "diverter");

        // Set motor directions
        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(CRServo.Direction.REVERSE);

        // Set brake behavior
        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

        isInitialized = true;

        if (telemetry != null) {
            telemetry.addData("[Launcher]", "Initialized");
        }
    }

    /**
     * Spin up launchers to target velocity
     * @param closeShot True for close shot velocity, false for far shot velocity
     */
    public void spinUp(boolean closeShot) {
        if (!isInitialized) return;

        if (closeShot) {
            targetVelocity = LAUNCHER_CLOSE_TARGET_VELOCITY;
            minVelocity = LAUNCHER_CLOSE_MIN_VELOCITY;
        } else {
            targetVelocity = LAUNCHER_FAR_TARGET_VELOCITY;
            minVelocity = LAUNCHER_FAR_MIN_VELOCITY;
        }

        leftLauncher.setVelocity(targetVelocity);
        rightLauncher.setVelocity(targetVelocity);
    }

    /**
     * Set custom target velocity
     * @param velocity Target velocity in RPM
     * @param minVel Minimum acceptable velocity in RPM
     */
    public void setTargetVelocity(double velocity, double minVel) {
        if (!isInitialized) return;

        targetVelocity = velocity;
        minVelocity = minVel;

        leftLauncher.setVelocity(targetVelocity);
        rightLauncher.setVelocity(targetVelocity);
    }

    /**
     * Check if launchers are ready (above minimum velocity)
     */
    public boolean isReady() {
        if (!isInitialized) return false;

        double leftVel = leftLauncher.getVelocity();
        double rightVel = rightLauncher.getVelocity();

        return leftVel > minVelocity && rightVel > minVelocity;
    }

    /**
     * Get current launcher velocities
     * @return Array [leftVelocity, rightVelocity]
     */
    public double[] getVelocities() {
        if (!isInitialized) return new double[]{0, 0};

        return new double[]{
            leftLauncher.getVelocity(),
            rightLauncher.getVelocity()
        };
    }

    /**
     * Feed artifact from specified side
     * Non-blocking - call update() to check if feeding is complete
     * @param side Which side to feed from
     */
    public void startFeed(LauncherSide side) {
        if (!isInitialized) return;

        if (side == LauncherSide.LEFT || side == LauncherSide.BOTH) {
            leftFeeder.setPower(FULL_SPEED);
            leftFeederTimer.reset();
            leftFeeding = true;
        }

        if (side == LauncherSide.RIGHT || side == LauncherSide.BOTH) {
            rightFeeder.setPower(FULL_SPEED);
            rightFeederTimer.reset();
            rightFeeding = true;
        }
    }

    /**
     * Feed artifact from specified side (blocking)
     * @param side Which side to feed from
     * @param opModeActive Function to check if OpMode is still active
     */
    public void feed(LauncherSide side, java.util.function.BooleanSupplier opModeActive) {
        if (!isInitialized) return;

        startFeed(side);

        // Wait for feed time
        while (opModeActive.getAsBoolean() && (leftFeeding || rightFeeding)) {
            update();
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /**
     * Update feeder state (call this in your loop if using non-blocking feed)
     */
    public void update() {
        if (leftFeeding && leftFeederTimer.seconds() >= FEED_TIME_SECONDS) {
            leftFeeder.setPower(STOP_SPEED);
            leftFeeding = false;
        }

        if (rightFeeding && rightFeederTimer.seconds() >= FEED_TIME_SECONDS) {
            rightFeeder.setPower(STOP_SPEED);
            rightFeeding = false;
        }
    }

    /**
     * Stop feeding immediately
     */
    public void stopFeeders() {
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        leftFeeding = false;
        rightFeeding = false;
    }

    /**
     * Set diverter position to route artifacts to left or right launcher
     * @param side Which side to route to
     */
    public void setDiverter(LauncherSide side) {
        if (!isInitialized) return;

        if (side == LauncherSide.LEFT) {
            diverter.setPosition(LEFT_POSITION);
        } else if (side == LauncherSide.RIGHT) {
            diverter.setPosition(RIGHT_POSITION);
        }
    }

    /**
     * Stop all launcher motors
     */
    public void stop() {
        if (!isInitialized) return;

        leftLauncher.setVelocity(STOP_SPEED);
        rightLauncher.setVelocity(STOP_SPEED);
        stopFeeders();
    }

    /**
     * Check if currently feeding
     */
    public boolean isFeeding() {
        return leftFeeding || rightFeeding;
    }

    /**
     * Get feeding progress (0.0 to 1.0)
     */
    public double getFeedProgress() {
        if (leftFeeding) {
            return Math.min(leftFeederTimer.seconds() / FEED_TIME_SECONDS, 1.0);
        } else if (rightFeeding) {
            return Math.min(rightFeederTimer.seconds() / FEED_TIME_SECONDS, 1.0);
        }
        return 0.0;
    }

    /**
     * Check if launcher is initialized
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Update telemetry with launcher status
     */
    public void updateTelemetry() {
        if (telemetry == null || !isInitialized) return;

        double[] velocities = getVelocities();

        telemetry.addData("[Launcher]", isReady() ? "READY" : "Spinning up...");
        telemetry.addData("  Left Vel", "%.0f / %.0f RPM", velocities[0], targetVelocity);
        telemetry.addData("  Right Vel", "%.0f / %.0f RPM", velocities[1], targetVelocity);

        if (leftFeeding) {
            telemetry.addData("  Left Feeder", "FEEDING (%.1f%%)", getFeedProgress() * 100);
        }
        if (rightFeeding) {
            telemetry.addData("  Right Feeder", "FEEDING (%.1f%%)", getFeedProgress() * 100);
        }
    }
}
