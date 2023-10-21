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

package org.firstinspires.ftc.teamcode.templates;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of TensorFlow Object Detection,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */

@Autonomous(name="TETST", group="Templates")

public class Template_PixelVision extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private int zone = 3; // Default if Team Prop not found

    private int watchTime = 5; // Watch for 5 seconds

    /* Declare Camera Fields */
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private static final String TFOD_MODEL_ASSET = "vs_crown.tflite";

    private static final String[] LABELS = {
            "BlueCrown",
            "RedCrown"
    };

    private TfodProcessor tfod;

    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            runtime.reset();
            while (opModeIsActive() && (runtime.seconds() < watchTime)) {

                telemetryTfod();

                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

        telemetry.addData("Zone", zone);
        telemetry.update();

        // Run Autonomous Based on Team Prop Position
        if (zone == 1) {
            zoneOne();
        }
        else if (zone == 2) {
            zoneTwo();
        }
        else {
            zoneThree();
        }


        // Autonomous Finished
        telemetry.addData("Path", "Complete");
        telemetry.update();


    }   // end runOpMode()


    // CENTERSTAGE Methods for Zone Operations (placing Pixels on Spike Marks or Backdrop)
    public void zoneOne() {
        Pose2d startPose = new Pose2d(16.0, -62.5, Math.toRadians(0));
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeqLEFT = drive.trajectorySequenceBuilder(startPose)
                .back(20)
                .splineToConstantHeading(new Vector2d(.5, -34), Math.toRadians(180))
                .waitSeconds(1.5)
                .strafeLeft(4)
                .lineToConstantHeading(new Vector2d(60, -60))
                .build();
        drive.followTrajectorySequence(trajSeqLEFT);
    }

    public void zoneTwo() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16.0, -62.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeqMIDDLE = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(8, -45))
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(8, -32))
                .waitSeconds(1.5)
                .strafeLeft(10)
                .lineToConstantHeading(new Vector2d(60, -60))
                .build();
        drive.followTrajectorySequence(trajSeqMIDDLE);
    }

    public void zoneThree() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(16.0, -62.5, Math.toRadians(0));

        drive.setPoseEstimate(startPose);
        TrajectorySequence trajSeqRIGHT = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(22.5, -45))
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(22.5, -40))
                .waitSeconds(1.5)
                .strafeLeft(20)
                .lineToConstantHeading(new Vector2d(60, -60))
                .build();
        drive.followTrajectorySequence(trajSeqRIGHT);
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
            else if (x > 350) {
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