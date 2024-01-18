/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@Autonomous(name="RED-Right", group="Alpha")

public class RED_Right extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private int zone = 3; // Default if Team Prop not found

    private int watchTime = 3; // Watch for 5 seconds

    /* Declare Camera Fields */
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "vs_beacon.tflite";

    private static final String[] LABELS = {
            "BlueBeacon",
            "RedBeacon"
    };

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        DcMotor belt = hardwareMap.get(DcMotor.class, "belt");
        Servo bucket = hardwareMap.get(Servo.class, "bucket");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");

        initTfod();

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < watchTime)) {

                telemetryTfod();
                telemetry.update();

                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }
                sleep(20);
            }
        }


        telemetry.addData("Zone", zone);
        telemetry.update();

        // Run Autonomous Based on Team Prop Position
        if (zone == 1) {
            //With RED
            zoneOne();
        }
        else if (zone == 2) {
            zoneTwo();
        }
        else {
            zoneThree();
        }

        telemetry.addData("Road Runner Path", "Complete");
        telemetry.update();

        visionPortal.close();
    }


    // CENTERSTAGE Methods for Zone Operations (placing Pixels on Spike Marks or Backdrop)
    public void zoneOne() {
        DcMotor belt = hardwareMap.get(DcMotor.class, "belt");
        Servo bucket = hardwareMap.get(Servo.class, "bucket");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        Pose2d startPose = new Pose2d(16.0, -62.0, Math.toRadians(0));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeqLEFT = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, ()-> {
                    stopper.setPosition(0);
                })
                .lineToConstantHeading(new Vector2d(16, -35))
                .lineToConstantHeading(new Vector2d(3, -35))
                .lineToConstantHeading(new Vector2d(3, -38.5))
                .lineToConstantHeading(new Vector2d(45, -38.5))
                .lineToConstantHeading(new Vector2d(45, -28))
                .lineToConstantHeading(new Vector2d(53.5, -28))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(50, -28))
                .lineToConstantHeading(new Vector2d(53.5, -60))
                .addTemporalMarker(8, ()-> {
                    bucket.setPosition(.6);
                    belt.setPower(.6);
                    belt.setTargetPosition(750);
                    belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .addTemporalMarker(9, ()-> {
                    stopper.setPosition(.2);
                })
                .build();
        drive.followTrajectorySequence(trajSeqLEFT);


        //move motor
    }

    public void zoneTwo() {
        DcMotor belt = hardwareMap.get(DcMotor.class, "belt");
        Servo bucket = hardwareMap.get(Servo.class, "bucket");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16.0, -62.0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeqMIDDLE = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, ()-> {
                    stopper.setPosition(0);
                })
                .lineToConstantHeading(new Vector2d(16, -40))
                .lineToConstantHeading(new Vector2d(14, -32))
                .lineToConstantHeading(new Vector2d(14, -37))
                .lineToConstantHeading(new Vector2d(45, -37))
                .lineToConstantHeading(new Vector2d(45, -36))
                .lineToConstantHeading(new Vector2d(53.5, -37))
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(50, -37))
                .lineToConstantHeading(new Vector2d(53.5, -60))
                .addTemporalMarker(7, ()-> {
                    bucket.setPosition(.6);
                    belt.setPower(.6);
                    belt.setTargetPosition(750);
                    belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .addTemporalMarker(8, ()-> {
                    stopper.setPosition(.2);
                })
                .build();
        drive.followTrajectorySequence(trajSeqMIDDLE);


        //move motor
    }

    public void zoneThree() {
        DcMotor belt = hardwareMap.get(DcMotor.class, "belt");
        Servo bucket = hardwareMap.get(Servo.class, "bucket");
        Servo stopper = hardwareMap.get(Servo.class, "stopper");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16.0, -62.0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeqRIGHT = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, ()-> {
                    stopper.setPosition(0);
                })
                .lineToConstantHeading(new Vector2d(16, -40))
                .lineToConstantHeading(new Vector2d(25, -40))
                .lineToConstantHeading(new Vector2d(25, -45))
                .lineToConstantHeading(new Vector2d(45, -45))
                .lineToConstantHeading(new Vector2d(45, -43))
                .lineToConstantHeading(new Vector2d(53.5, -43))
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(50, -43))
                .lineToConstantHeading(new Vector2d(53.5, -60))
                .addTemporalMarker(6, ()-> {
                    bucket.setPosition(.6);
                    belt.setPower(.6);
                    belt.setTargetPosition(750);
                    belt.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                })
                .addTemporalMarker(7, ()-> {
                    stopper.setPosition(.2);
                })
                .build();
        drive.followTrajectorySequence(trajSeqRIGHT);

        //move motor
    }


    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(16.0 / 9.0)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.enableLiveView(true);
        builder.addProcessor(tfod);
        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.75f);
        visionPortal.setProcessorEnabled(tfod, true);
    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            if (x < 300) {
                zone = 1;
            }
            else if (x > 300) {
                zone = 2;
            }
            else {
                zone = 3;
            }

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

    }   // end method telemetryTfod()
}   // end class