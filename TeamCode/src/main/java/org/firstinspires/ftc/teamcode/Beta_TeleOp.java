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

        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightRearMotor = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor leftRearMotor = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor leftSpin = hardwareMap.get(DcMotor.class, "leftSpin");
        DcMotor rightSpin = hardwareMap.get(DcMotor.class, "rightSpin");
        DcMotor leftArm = hardwareMap.get(DcMotor.class, "leftArm");
        DcMotor rightArm = hardwareMap.get(DcMotor.class, "rightArm");
        Servo bucket = hardwareMap.get(Servo.class, "bucket");
        Servo bucketStop = hardwareMap.get(Servo.class, "bucketStop");
        Servo launcher = hardwareMap.get(Servo.class, "launcher");

        double speed;
        double position = 0;
        double armpower = 0;
        boolean direction = true;
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketStop.setPosition(.1);
        waitForStart();
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (opModeIsActive()) {
            //Speed Control
            if (gamepad1.left_trigger > .1) {
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

            if(gamepad2.dpad_up) {
                direction = true;
            }
            if(gamepad2.dpad_down) {
                direction = false;
            }
            if(direction == false) {
                if(leftArm.getCurrentPosition() < 25) {
                    bucket.setPosition(.3);
                } else {
                    bucket.setPosition(.1);
                }
            }
            if(direction == true) {
                if(leftArm.getCurrentPosition() < 25) {
                    bucket.setPosition(.1);
                } else {
                    bucket.setPosition(.5);
                }
            }
            armpower = (gamepad2.right_stick_y / 3) + .1;
            if (armpower < 0 && leftArm.getCurrentPosition() < 100) {
                armpower = -.05;
            }
            if(leftArm.getCurrentPosition() > 135) {
                armpower = -.2 - Math.abs(gamepad2.right_stick_y);
            } else if(leftArm.getCurrentPosition() < 135 && leftArm.getCurrentPosition() > 100) {
                armpower = gamepad2.right_stick_y / 2;
            }

                leftArm.setPower(armpower);
                rightArm.setPower(armpower);

                if(gamepad1.left_bumper) {
                    leftSpin.setPower(.7);
                    rightSpin.setPower(-.7);
                }
                if(gamepad1.right_bumper) {
                    leftSpin.setPower(0);
                    rightSpin.setPower(0);
                }

                if (gamepad2.a) {
                    bucketStop.setPosition(.6);
                }
                if (gamepad2.x) {
                    bucketStop.setPosition(.1);
                }
                if (gamepad2.left_bumper && gamepad2.right_bumper) {
                    launcher.setPosition(.3);
                }


                telemetry.addData("position", leftArm.getCurrentPosition());
//                telemetry.addData("servoposition", position);
                telemetry.addData("power", leftArm.getPowerFloat());
                telemetry.update();
            }

            rightFrontMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightRearMotor.setPower(0.0);
            leftRearMotor.setPower(0.0);

        }


    }