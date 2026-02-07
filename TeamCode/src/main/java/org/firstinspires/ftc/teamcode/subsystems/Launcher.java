package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.RobotEnums.LauncherSide;

/**
 * Launcher subsystem. Controls launcher motors and feeder servos.
 */
public class Launcher {

    // Velocity targets (RPM)
    public static final double LAUNCHER_CLOSE_TARGET_VELOCITY = 1150;
    public static final double LAUNCHER_CLOSE_MIN_VELOCITY = 1100;
    public static final double LAUNCHER_FAR_TARGET_VELOCITY = 1540;
    public static final double LAUNCHER_FAR_MIN_VELOCITY = 1450;

    private static final double FEED_TIME_SECONDS = 0.80;
    private static final double STOP_SPEED = 0.0;
    private static final double FULL_SPEED = 1.0;
    private DcMotorEx leftLauncher;
    private DcMotorEx rightLauncher;
    private CRServo leftFeeder;
    private CRServo rightFeeder;
    private Telemetry telemetry;

    private boolean isInitialized = false;
    private double targetVelocity = LAUNCHER_CLOSE_TARGET_VELOCITY;
    private double minVelocity = LAUNCHER_CLOSE_MIN_VELOCITY;
    private ElapsedTime leftFeederTimer = new ElapsedTime();
    private ElapsedTime rightFeederTimer = new ElapsedTime();
    private boolean leftFeeding = false;
    private boolean rightFeeding = false;

    public Launcher(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        leftLauncher = hardwareMap.get(DcMotorEx.class, "leftLauncher");
        rightLauncher = hardwareMap.get(DcMotorEx.class, "rightLauncher");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");

        leftLauncher.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFeeder.setDirection(CRServo.Direction.REVERSE);

        leftLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        rightLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        isInitialized = true;
    }

    /** Spin up to close or far shot velocity. */
    public void spinUp(boolean closeShot) {
        if (!isInitialized) return;
        if (closeShot) {
            targetVelocity = LAUNCHER_CLOSE_TARGET_VELOCITY;

        } else {
            targetVelocity = LAUNCHER_FAR_TARGET_VELOCITY;

        }
        leftLauncher.setVelocity(targetVelocity);
        rightLauncher.setVelocity(targetVelocity);
    }

    /** Set custom velocity. */
    public void setTargetVelocity(double velocity, double minVel) {
        if (!isInitialized) return;
        targetVelocity = velocity;
        minVelocity = minVel;
        leftLauncher.setVelocity(targetVelocity);
        rightLauncher.setVelocity(targetVelocity);
    }

    public boolean isReady() {
        if (!isInitialized) return false;
        return leftLauncher.getVelocity() > minVelocity && rightLauncher.getVelocity() > minVelocity;
    }

    public double[] getVelocities() {
        if (!isInitialized) return new double[]{0, 0};
        return new double[]{leftLauncher.getVelocity(), rightLauncher.getVelocity()};
    }

    /** Start feeding (non-blocking). Call update() to auto-stop. */
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

    /** Blocking feed. */
    public void feed(LauncherSide side, java.util.function.BooleanSupplier opModeActive) {
        if (!isInitialized) return;
        startFeed(side);
        while (opModeActive.getAsBoolean() && (leftFeeding || rightFeeding)) {
            update();
            try { Thread.sleep(10); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
        }
    }

    /** Update feeder timers. Call in main loop. */
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

    public void stopFeeders() {
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        leftFeeding = false;
        rightFeeding = false;
    }

    public void stop() {
        if (!isInitialized) return;
        leftLauncher.setVelocity(STOP_SPEED);
        rightLauncher.setVelocity(STOP_SPEED);
        stopFeeders();
    }

    public boolean isFeeding() { return leftFeeding || rightFeeding; }

    public double getFeedProgress() {
        if (leftFeeding) return Math.min(leftFeederTimer.seconds() / FEED_TIME_SECONDS, 1.0);
        if (rightFeeding) return Math.min(rightFeederTimer.seconds() / FEED_TIME_SECONDS, 1.0);
        return 0.0;
    }

    public boolean isInitialized() { return isInitialized; }

    public void updateTelemetry() {
        if (telemetry == null || !isInitialized) return;
        double[] vel = getVelocities();
        telemetry.addData("[Launcher]", isReady() ? "READY" : "Spinning...");
        telemetry.addData("  L/R RPM", "%.0f / %.0f (target: %.0f)", vel[0], vel[1], targetVelocity);
        if (isFeeding()) telemetry.addData("  Feeding", "%.0f%%", getFeedProgress() * 100);
    }
}
