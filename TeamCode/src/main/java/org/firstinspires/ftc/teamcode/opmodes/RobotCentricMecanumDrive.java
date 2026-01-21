package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class RobotCentricMecanumDrive extends LinearOpMode {
    double leftlauncherpower=0;
    double rightLauncherpower=0;
    double launcherPower = 0.5;
    boolean lastA = false;
    boolean lastB = false;

    boolean lastLeftBumper = false;
    boolean lastRightBumper = false;

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor intakeMotor = null;
    private DcMotor rightLauncher = null;
    private DcMotor leftLauncher = null;
    private CRServo rightFeeder = null;
    private CRServo leftFeeder = null;
    private Servo diverter;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");
        diverter = hardwareMap.get(Servo.class, "diverter");

        //reverse later
<<<<<<< Updated upstream
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
=======
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
>>>>>>> Stashed changes

        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);

        leftFeeder.setDirection(CRServo.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        diverter.setPosition(1);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontDrive.setPower(frontLeftPower);
            leftBackDrive.setPower(backLeftPower);
            rightFrontDrive.setPower(frontRightPower);
            rightBackDrive.setPower(backRightPower);

            intake();
            launching();
            updateTelemetry();
        }

    }

    private void intake() {
        if (gamepad1.left_bumper) {
            intakeMotor.setPower(1);
        } else if (gamepad1.right_bumper) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
    }

    private void launching() {

        if (gamepad1.left_trigger > 0) {
            leftFeeder.setPower(1);
        } else {
            leftFeeder.setPower(0);
        }

        if (gamepad1.right_trigger > 0) {
            rightFeeder.setPower(1);
        } else {
            rightFeeder.setPower(0);
        }

        boolean currentA = gamepad2.a;
        boolean currentB = gamepad2.b;

        if (currentA && !lastA) {
            launcherPower += 0.05;
        }

        if (currentB && !lastB) {
            launcherPower -= 0.05;
        }

        launcherPower = Math.max(0.0, Math.min(1.0, launcherPower));

        leftlauncherpower = launcherPower;
        rightLauncherpower = launcherPower;

        lastA = currentA;
        lastB = currentB;

        leftLauncher.setPower(leftlauncherpower);
        rightLauncher.setPower(rightLauncherpower);

    }


    private void updateTelemetry(){
        telemetry.addData("Left Launcher Power", "%.2f", leftlauncherpower);
        telemetry.addData("Right Launcher Power", "%.2f", rightLauncherpower);

        telemetry.addData("Left Feeder", gamepad1.left_trigger > 0 ? "ON" : "OFF");
        telemetry.addData("Right Feeder", gamepad1.right_trigger > 0 ? "ON" : "OFF");

        telemetry.update();
    }
}