package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class RobotCentricMecanumDrive extends LinearOpMode{
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightBack = null;
    private DcMotor rightFront = null;
    private DcMotor intakeMotor = null;
    private DcMotor rightLauncher = null;
    private DcMotor leftLauncher = null;
    private CRServo rightFeeder= null;
    private CRServo leftFeeder= null;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightLauncher = hardwareMap.get(DcMotor.class, "rightLauncher");
        leftLauncher = hardwareMap.get(DcMotor.class, "leftLauncher");
        rightFeeder = hardwareMap.get(CRServo.class, "rightFeeder");
        leftFeeder = hardwareMap.get(CRServo.class, "leftFeeder");

        //reverse later
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightLauncher.setDirection(DcMotor.Direction.FORWARD);
        leftLauncher.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
            rightFront.setPower(rightFrontPower);

            intake();
            launching();
        }

    }

    private void intake(){
        if (gamepad1.a) {
            intakeMotor.setPower(0.5);
        } else if (gamepad1.y) {
            intakeMotor.setPower(-0.5);
        } else {
            intakeMotor.setPower(0);
        }
    }

    private void launching(){
        if (gamepad1.left_trigger>0) {
            leftFeeder.setPower(gamepad1.left_trigger);
            leftLauncher.setPower(1);
        }
        else {
         leftFeeder.setPower(0);
         leftLauncher.setPower(0);
        }
        if(gamepad1.right_trigger>0){
            rightLauncher.setPower(gamepad1.right_trigger);
            rightFeeder.setPower(1);
        }
        else{
            rightLauncher.setPower(0);
            rightFeeder.setPower(0);
        }
    }

}
