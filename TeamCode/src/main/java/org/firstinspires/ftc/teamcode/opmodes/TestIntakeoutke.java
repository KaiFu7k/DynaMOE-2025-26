package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestIntakeoutke extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor left_motor  = hardwareMap.get(DcMotor.class, "left_motor");
        DcMotor right_motor   = hardwareMap.get(DcMotor.class, "right_motor");


        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(gamepad1.left_trigger > 0){
                right_motor.setDirection(DcMotor.Direction.REVERSE);
                left_motor.setDirection(DcMotor.Direction.FORWARD);

                left_motor.setPower(gamepad1.left_trigger);
                right_motor.setPower(gamepad1.left_trigger);
            }

            if(gamepad1.right_trigger > 0){
                left_motor.setDirection(DcMotor.Direction.REVERSE);
                right_motor.setDirection(DcMotor.Direction.FORWARD);

                left_motor.setPower(gamepad1.right_trigger);
                right_motor.setPower(gamepad1.right_trigger);
            }

//            left_motor.setPower(gamepad1.left_trigger);
//            right_motor.setPower(-gamepad1.left_trigger);
//            left_motor.setPower(gamepad1.right_trigger);
//            right_motor.setPower(-gamepad1.right_trigger);

        }
    }
}
