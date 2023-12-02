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
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
//        Servo launcher = hardwareMap.get(Servo.class, "launcher");
        DcMotorEx bucket = hardwareMap.get(DcMotorEx.class, "bucket");
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "arm");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotor.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bucket.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if(gamepad1.left_bumper) {
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

            if(gamepad2.left_bumper) {
                intake.setPower(-1);
            }
            if(gamepad2.right_bumper) {
                intake.setPower(0);
            }

            double armSpeed = gamepad2.left_stick_y;
            arm.setPower(armSpeed);



            double bucketSpeed = gamepad2.right_stick_y;
            bucket.setPower(bucketSpeed);


            telemetry.addData("Arm Pos", arm.getCurrentPosition());
            telemetry.addData("Bucket Pos", bucket.getCurrentPosition());
            telemetry.update();

//            if (gamepad2.) {
//                bucket.setTargetPosition(0);
//                bucket.setPower(.2);
//            }
//            if (gamepad2.x) {
//                bucket.setTargetPosition(21331);
//                bucket.setPower(.2);
//            }
        }
             arm.setPower(0.0);
             bucket.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightRearMotor.setPower(0.0);
            leftRearMotor.setPower(0.0);


        }


    }