package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Beta_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFront");
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFront");
        DcMotor rightRearMotor = hardwareMap.get(DcMotor.class,"rightRear");
        DcMotor leftRearMotor = hardwareMap.get(DcMotor.class,"leftRear");
        DcMotor leftSpin = hardwareMap.get(DcMotor.class,"leftSpin");
        DcMotor rightSpin = hardwareMap.get(DcMotor.class,"rightSpin");
        DcMotor leftArm = hardwareMap.get(DcMotor.class,"leftArm");
        DcMotor rightArm = hardwareMap.get(DcMotor.class,"rightArm");
        Servo bucket = hardwareMap.get(Servo.class,"bucket");
        Servo bucketStop = hardwareMap.get(Servo.class,"bucketStop");

        double speed;
        double position = 0;
        waitForStart();
        while (opModeIsActive()) {
            //Speed Control
            if (gamepad1.right_bumper) {
                speed = 1;
            } else if (gamepad1.left_bumper) {
                speed = 0.3;
            } else {
                speed = 0.6;
            }

            //Controller Input - Moves Drive Train
            double vertical = gamepad1.left_stick_y * speed;
            double horizontal = gamepad1.left_stick_x * speed;
            double pivot = -gamepad1.right_stick_x * speed; //normal has negative

            rightFrontMotor.setPower(-pivot + vertical + horizontal);
            leftFrontMotor.setPower(-pivot - vertical + horizontal);
            rightRearMotor.setPower(-pivot + vertical - horizontal);
            leftRearMotor.setPower(-pivot - vertical - horizontal);

            double armpower = (gamepad2.right_stick_y / 4) + .1;
            leftArm.setPower(armpower);
            rightArm.setPower(armpower);

            if(leftArm.getCurrentPosition() > 10 && leftArm.getCurrentPosition() < 50) {
                bucket.setPosition(0);
            } else if(leftArm.getCurrentPosition() > 50) {
                bucket.setPosition(.7);
            } else {
                bucket.setPosition(.2);
            }

            if(gamepad2.a) {
                bucketStop.setPosition(.6);
            }
            if(gamepad2.x) {
                bucketStop.setPosition(0);
            }
            if(gamepad2.y) {
                bucket.setPosition(position);
            }

            if(gamepad2.dpad_up) {
                position += .1;
                sleep(500);
            }

            if(gamepad2.dpad_down) {
                position -= .1;
                sleep(500);
            }
            telemetry.addData("position",leftArm.getCurrentPosition());
            telemetry.addData("target",leftArm.getTargetPosition());
            telemetry.addData("position",position);
            telemetry.addData("power",armpower);
            telemetry.update();
        }

        rightFrontMotor.setPower(0.0);
        leftFrontMotor.setPower(0.0);
        rightRearMotor.setPower(0.0);
        leftRearMotor.setPower(0.0);

    }


}