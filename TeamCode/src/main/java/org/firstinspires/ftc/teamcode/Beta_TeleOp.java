package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class Beta_TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightRearMotor = hardwareMap.get(DcMotor.class, "rightRear");
        DcMotor leftRearMotor = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        Servo shooter = hardwareMap.get(Servo.class, "shooter");
        Servo bucket = hardwareMap.get(Servo.class, "bucket");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        DcMotor hang = hardwareMap.get(DcMotor.class, "hang");
        DcMotorEx hangArm = hardwareMap.get(DcMotorEx.class, "hangArm");
        DcMotor belt = hardwareMap.get(DcMotor.class, "belt");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);

        belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setVelocityPIDFCoefficients(13,0,0,0);
        waitForStart();
        stopper.setPosition(.2);
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if(gamepad1.left_trigger > 0) {
                y *= 0.3;
                x *= 0.3;
                rx *= 0.3;
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            leftFrontMotor.setPower(frontLeftPower);
            leftRearMotor.setPower(backLeftPower);
            rightFrontMotor.setPower(frontRightPower);
            rightRearMotor.setPower(backRightPower);

            if(gamepad1.left_bumper) {
                intake.setPower(-1);
            }
            if(gamepad1.right_bumper) {
                intake.setPower(0);
            }
            if(gamepad2.dpad_up) {
                hang.setPower(-.7);
            }
            if(gamepad2.dpad_down) {
                hang.setPower(.7);
            }
            if(gamepad2.x && gamepad2.dpad_right) {
                hangArm.setTargetPosition(-130);
                hangArm.setPower(.6);
                hangArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.left_bumper && gamepad2.right_bumper) {
                shooter.setPosition(0);
            }
            if(belt.getCurrentPosition() > 100) {
                bucket.setPosition(.6);
            } else {
                bucket.setPosition(.3);
            }
            if(gamepad2.a) {
                stopper.setPosition(.2);
            }
            if(gamepad2.right_trigger > 0) {
                stopper.setPosition(0);
                belt.setPower(.6);
                belt.setTargetPosition(1500);
                belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad2.left_trigger > 0) {
                stopper.setPosition(.2);
                belt.setPower(.6);
                belt.setTargetPosition(0);
                belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("hangarm",hangArm.getCurrentPosition());
            telemetry.update();
        }
            belt.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightRearMotor.setPower(0.0);
            leftRearMotor.setPower(0.0);


        }


    }