package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="TESTRED", preselectTeleOp= "Beta_TeleOp")
public class TEST extends LinearOpMode {
    boolean LEFT = false;
    boolean MIDDLE = false;
    boolean RIGHT = false;
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "vs_crown.tflite";
    private static final String[] LABELS = {
            "BlueCrown",
            "RedCrown"
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        Log.d("Autonomous", "runOpMode: ");
        initTfod();
        Log.d("Autonomous", "2 ");
        getTfodResults();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(16.0, -62.5, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (LEFT) {
            TrajectorySequence trajSeqLEFT = drive.trajectorySequenceBuilder(startPose)
                    .back(20)
                    .splineToConstantHeading(new Vector2d(.5, -34), Math.toRadians(180))
                    .waitSeconds(1.5)
                    .strafeLeft(4)
                    .lineToConstantHeading(new Vector2d(60, -60))
                    .build();
            drive.followTrajectorySequence(trajSeqLEFT);
        }
        if (MIDDLE) {
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
        if (RIGHT) {
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
    }

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
    private void getTfodResults() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;

            if (x < 300) {
                LEFT = true;
                telemetry.addData("Left", "");
            }
            if (x > 350) {
                MIDDLE = true;
                telemetry.addData("Middle", "");
            } else {
                RIGHT = true;
                telemetry.addData("Right", "");
            }
        }
    }
}
