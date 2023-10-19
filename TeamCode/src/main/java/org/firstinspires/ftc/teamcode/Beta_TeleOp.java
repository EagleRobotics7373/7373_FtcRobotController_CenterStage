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
        DcMotorEx leftArm = hardwareMap.get(DcMotorEx.class,"leftArm");
        DcMotorEx rightArm = hardwareMap.get(DcMotorEx.class,"rightArm");
        Servo bucket = hardwareMap.get(Servo.class,"bucket");
        Servo bucketStop = hardwareMap.get(Servo.class,"bucketStop");

        rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double speed;
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

            if(gamepad2.x) {
                leftArm.setTargetPosition(50);
                rightArm.setTargetPosition(50);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.y) {
                leftArm.setTargetPosition(100);
                rightArm.setTargetPosition(100);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.b) {
                leftArm.setTargetPosition(150);
                rightArm.setTargetPosition(150);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.a) {
                leftArm.setTargetPosition(200);
                rightArm.setTargetPosition(200);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.dpad_down) {
                leftArm.setTargetPosition(250);
                rightArm.setTargetPosition(250);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.dpad_left) {
                leftArm.setTargetPosition(300);
                rightArm.setTargetPosition(300);
                leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.dpad_right) {
                leftArm.setTargetPosition(0);
                rightArm.setTargetPosition(0);
            }
            if(gamepad2.dpad_up) {
                leftArm.setTargetPosition(0);
                rightArm.setTargetPosition(0);
            }
            telemetry.addData("position",leftArm.getCurrentPosition());
            telemetry.addData("target",leftArm.getTargetPosition());
            telemetry.update();
        }

        rightFrontMotor.setPower(0.0);
        leftFrontMotor.setPower(0.0);
        rightRearMotor.setPower(0.0);
        leftRearMotor.setPower(0.0);

    }


}