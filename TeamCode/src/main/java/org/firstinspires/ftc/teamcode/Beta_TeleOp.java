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
            double pivot = -gamepad1.right_stick_y * speed; //normal has negative

            rightFrontMotor.setPower(-pivot + vertical + horizontal);
            leftFrontMotor.setPower(-pivot - vertical + horizontal);
            rightRearMotor.setPower(-pivot + vertical - horizontal);
            leftRearMotor.setPower(-pivot - vertical - horizontal);

            leftArm.setPower(gamepad2.left_stick_y);
            rightArm.setPower(gamepad2.left_stick_y);
        }

        rightFrontMotor.setPower(0.0);
        leftFrontMotor.setPower(0.0);
        rightRearMotor.setPower(0.0);
        leftRearMotor.setPower(0.0);

    }


}