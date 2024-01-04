package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Distance Testing")
public class DistanceTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance: ", distanceSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
