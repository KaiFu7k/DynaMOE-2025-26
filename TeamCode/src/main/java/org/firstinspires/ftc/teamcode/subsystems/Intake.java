package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake subsystem for collecting artifacts
 * Controls intake motor
 * Used by both Autonomous and TeleOp
 */
public class Intake {

    // Constants
    public static final double DEFAULT_INTAKE_POWER = 0.5;
    public static final double DEFAULT_OUTTAKE_POWER = -0.5;

    // Hardware
    private DcMotor intakeMotor;

    // Telemetry
    private Telemetry telemetry;

    // State tracking
    private boolean isInitialized = false;
    private double currentPower = 0.0;

    /**
     * Constructor
     * @param telemetry Telemetry object for debug output
     */
    public Intake(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Initialize hardware from HardwareMap
     */
    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");

        // Set motor direction
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set brake behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        isInitialized = true;

        if (telemetry != null) {
            telemetry.addData("[Intake]", "Initialized");
        }
    }

    /**
     * Run intake at specified power
     * @param power Power level (-1.0 to 1.0)
     */
    public void run(double power) {
        if (!isInitialized) return;

        currentPower = power;
        intakeMotor.setPower(power);
    }

    /**
     * Run intake at default power (collecting artifacts)
     */
    public void intake() {
        run(DEFAULT_INTAKE_POWER);
    }

    /**
     * Run intake in reverse at default power (ejecting artifacts)
     */
    public void outtake() {
        run(DEFAULT_OUTTAKE_POWER);
    }

    /**
     * Stop intake motor
     */
    public void stop() {
        run(0.0);
    }

    /**
     * Get current intake power
     */
    public double getCurrentPower() {
        return currentPower;
    }

    /**
     * Check if intake is running
     */
    public boolean isRunning() {
        return Math.abs(currentPower) > 0.01;
    }

    /**
     * Check if intake is initialized
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Update telemetry with intake status
     */
    public void updateTelemetry() {
        if (telemetry == null || !isInitialized) return;

        String state = "Stopped";
        if (currentPower > 0.01) {
            state = "Intaking";
        } else if (currentPower < -0.01) {
            state = "Outtaking";
        }

        telemetry.addData("[Intake]", state);
        telemetry.addData("  Power", "%.2f", currentPower);
    }
}
