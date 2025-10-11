package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "4 Motor Test")
public class SimpleFourMotorAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        // make the names of the motors
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBack   = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");


        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        // power test

        double rightPower = 0.6;
        double leftPower = 0.6;

        // driivng
        telemetry.addData("status", "driving");
        telemetry.addData("right", rightPower);
        telemetry.addData("left", leftPower);
        telemetry.update();

        leftFront.setPower(leftPower);
        leftBack.setPower(leftPower);
        rightBack.setPower(rightPower);
        rightFront.setPower(rightPower);

        // driving for x secs
        sleep(2000);

        // stop
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);
        telemetry.addData("status", "complete");
        telemetry.update();
    }
}