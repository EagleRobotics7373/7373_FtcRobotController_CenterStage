package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


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
        Servo bucket2 = hardwareMap.get(Servo.class, "bucket");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        DcMotor hang = hardwareMap.get(DcMotor.class, "hang");
        DcMotorEx hangArm = hardwareMap.get(DcMotorEx.class, "hangArm");
        DcMotor belt = hardwareMap.get(DcMotor.class, "belt");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        RevBlinkinLedDriver ledLights = hardwareMap.get(RevBlinkinLedDriver.class,"ledLights");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);

        hangArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangArm.setVelocityPIDFCoefficients(20,0,0,0);
        waitForStart();
        stopper.setPosition(.2);
        while (opModeIsActive()) {
//            double y = gamepad1.left_stick_x * .8;
//            double x = gamepad1.left_stick_y * .8;
//            double rx = gamepad1.right_stick_x * .8;
//
//            if(gamepad1.left_trigger > 0) {
//                y *= .5;
//                x *= .5;
//                rx *= .5;
//            }
//            if(gamepad1.back) {
//                imu.resetYaw();
//            }
//            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
//
//            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
//            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
//
//            rotX = rotX * 1.1;
//
//            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
//
//            leftFrontMotor.setPower(frontLeftPower);
//            leftRearMotor.setPower(backLeftPower);
//            rightFrontMotor.setPower(frontRightPower);
//            rightRearMotor.setPower(backRightPower);
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            y *= .5;
            x *= .5;
            rx *= .5;
            if(gamepad1.left_trigger > 0) {
                y *= .5;
                x *= .5;
                rx *= .5;
            }
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
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
                intake.setPower(-.6);
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

            if(hangArm.getCurrentPosition() < -135)  {
                hangArm.setPower(-.6);
            }

            if(gamepad2.left_bumper && gamepad2.right_bumper) {
                shooter.setPosition(0);
            }

            if(belt.getCurrentPosition() > 100) {
                bucket2.setPosition(.1);
            } else {
                bucket2.setPosition(.4);
            }

            if(gamepad2.a) {
                stopper.setPosition(.3);
            }
            //./adb connect 192.168.43.1:5555
            if(gamepad2.right_trigger > 0) {
                stopper.setPosition(0);
                belt.setPower(.6);
                belt.setTargetPosition(1500);
                belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.left_trigger > 0) {
                stopper.setPosition(.3);
                belt.setPower(.6);
                belt.setTargetPosition(0);
                belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad2.back) {
                belt.setPower(-.4);
                sleep(1000);
                belt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (distanceSensor.getDistance(DistanceUnit.INCH) < 8) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (distanceSensor.getDistance(DistanceUnit.INCH) > 8.01 && distanceSensor.getDistance(DistanceUnit.INCH) < 10 ) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            telemetry.addData("belt",belt.getCurrentPosition());
            telemetry.update();
        }
            belt.setPower(0.0);
            rightFrontMotor.setPower(0.0);
            leftFrontMotor.setPower(0.0);
            rightRearMotor.setPower(0.0);
            leftRearMotor.setPower(0.0);
        }


    }