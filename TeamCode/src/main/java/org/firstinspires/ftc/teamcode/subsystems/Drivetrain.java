package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Drivetrain subsystem for mecanum drive
 * Handles initialization and control of all 4 drive motors
 * Used by both Autonomous and TeleOp
 */
public class Drivetrain {

    // Hardware
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Telemetry
    private Telemetry telemetry;

    // State tracking
    private boolean isInitialized = false;

    /**
     * Constructor
     * @param telemetry Telemetry object for debug output
     */
    public Drivetrain(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /**
     * Initialize hardware from HardwareMap
     * Must be called during OpMode initialization
     */
    public void init(HardwareMap hardwareMap) {
        // Initialize motors
        leftFrontDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // Set motor directions (standard FTC mecanum configuration)
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set brake behavior
        setBrakeMode(true);

        isInitialized = true;

        if (telemetry != null) {
            telemetry.addData("[Drivetrain]", "Initialized");
        }
    }

    /**
     * Set power to all 4 motors individually
     * @param leftFront Power for left front motor (-1.0 to 1.0)
     * @param rightFront Power for right front motor (-1.0 to 1.0)
     * @param leftBack Power for left back motor (-1.0 to 1.0)
     * @param rightBack Power for right back motor (-1.0 to 1.0)
     */
    public void setPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        if (!isInitialized) {
            if (telemetry != null) {
                telemetry.addData("[Drivetrain]", "ERROR: Not initialized!");
            }
            return;
        }

        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }

    /**
     * Stop all drive motors
     */
    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    /**
     * Set brake or coast mode for all motors
     * @param brake True for brake mode, false for coast mode
     */
    public void setBrakeMode(boolean brake) {
        if (!isInitialized) return;

        DcMotor.ZeroPowerBehavior behavior = brake ?
            DcMotor.ZeroPowerBehavior.BRAKE :
            DcMotor.ZeroPowerBehavior.FLOAT;

        leftFrontDrive.setZeroPowerBehavior(behavior);
        rightFrontDrive.setZeroPowerBehavior(behavior);
        leftBackDrive.setZeroPowerBehavior(behavior);
        rightBackDrive.setZeroPowerBehavior(behavior);
    }

    /**
     * Get access to individual motors (for Pedro Pathing or other advanced control)
     */
    public DcMotor getLeftFrontDrive() { return leftFrontDrive; }
    public DcMotor getRightFrontDrive() { return rightFrontDrive; }
    public DcMotor getLeftBackDrive() { return leftBackDrive; }
    public DcMotor getRightBackDrive() { return rightBackDrive; }

    /**
     * Check if drivetrain is initialized
     */
    public boolean isInitialized() {
        return isInitialized;
    }

    /**
     * Update telemetry with drivetrain status
     */
    public void updateTelemetry() {
        if (telemetry == null || !isInitialized) return;

        telemetry.addData("[Drivetrain]", "Ready");
        telemetry.addData("  LF Power", "%.2f", leftFrontDrive.getPower());
        telemetry.addData("  RF Power", "%.2f", rightFrontDrive.getPower());
        telemetry.addData("  LB Power", "%.2f", leftBackDrive.getPower());
        telemetry.addData("  RB Power", "%.2f", rightBackDrive.getPower());
    }
}
