package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake subsystem for collecting artifacts.
 */
public class Intake {

    public static final double DEFAULT_INTAKE_POWER = 1;
    public static final double DEFAULT_OUTTAKE_POWER = -1;

    private DcMotor intakeMotor;
    private Telemetry telemetry;
    private boolean isInitialized = false;
    private double currentPower = 0.0;

    public Intake(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        isInitialized = true;
    }

    public void run(double power) {
        if (!isInitialized) return;
        currentPower = power;
        intakeMotor.setPower(power);
    }

    public void intake() { run(DEFAULT_INTAKE_POWER); }
    public void outtake() { run(DEFAULT_OUTTAKE_POWER); }
    public void stop() { run(0.0); }

    public double getCurrentPower() { return currentPower; }
    public boolean isRunning() { return Math.abs(currentPower) > 0.01; }
    public boolean isInitialized() { return isInitialized; }

    public void updateTelemetry() {
        if (telemetry == null || !isInitialized) return;
        String state = currentPower > 0.01 ? "Intaking" : (currentPower < -0.01 ? "Outtaking" : "Stopped");
        telemetry.addData("[Intake]", "%s (%.2f)", state, currentPower);
    }
}
