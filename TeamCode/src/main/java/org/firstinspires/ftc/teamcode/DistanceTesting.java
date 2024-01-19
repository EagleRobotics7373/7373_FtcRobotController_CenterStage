package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Distance Testing")
public class DistanceTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        RevBlinkinLedDriver ledLights = hardwareMap.get(RevBlinkinLedDriver.class,"ledLights");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();

            if (distanceSensor.getDistance(DistanceUnit.INCH) < 8) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            } else if (distanceSensor.getDistance(DistanceUnit.INCH) > 8.01 && distanceSensor.getDistance(DistanceUnit.INCH) < 10 ) {
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            } else
                ledLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
    }
}
