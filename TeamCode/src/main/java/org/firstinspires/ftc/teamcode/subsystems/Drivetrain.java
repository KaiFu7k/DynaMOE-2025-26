package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Mecanum drivetrain subsystem. Controls all 4 drive motors.
 */
public class Drivetrain {

    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private Telemetry telemetry;
    private boolean isInitialized = false;

    public Drivetrain(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap) {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // Standard FTC mecanum configuration
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setBrakeMode(true);
        isInitialized = true;
    }

    /** Set power to all 4 motors (-1.0 to 1.0). */
    public void setPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        if (!isInitialized) return;
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    public void setBrakeMode(boolean brake) {
        if (!isInitialized) return;
        DcMotor.ZeroPowerBehavior behavior = brake ?
            DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    public DcMotor getLeftFrontDrive() { return leftFrontDrive; }
    public DcMotor getRightFrontDrive() { return rightFrontDrive; }
    public DcMotor getLeftBackDrive() { return leftBackDrive; }
    public DcMotor getRightBackDrive() { return rightBackDrive; }
    public boolean isInitialized() { return isInitialized; }

    public void updateTelemetry() {
        if (telemetry == null || !isInitialized) return;
        telemetry.addData("[Drivetrain]", "Ready");
        telemetry.addData("  LF/RF", "%.2f / %.2f", leftFrontDrive.getPower(), rightFrontDrive.getPower());
        telemetry.addData("  LB/RB", "%.2f / %.2f", leftBackDrive.getPower(), rightBackDrive.getPower());
    }
}
