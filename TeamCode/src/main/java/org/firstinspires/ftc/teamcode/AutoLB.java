package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
@Disabled
@Autonomous(name="Auto Scoring Zone Blue", preselectTeleOp= "Beta_TeleOp")
public class AutoLB extends LinearOpMode {
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
        initTfod();
        SampleMecanumDrive drive =new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(16.0, 62.5, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        //Edit the sequences here
        TrajectorySequence trajSeqLEFT = drive.trajectorySequenceBuilder(startPose)
                .back(20)
                .splineToConstantHeading(new Vector2d(.5, 34), Math.toRadians(180))
                .waitSeconds(1.5)
                .forward(4)
                .lineToConstantHeading(new Vector2d(60, 60))
                .build();
        TrajectorySequence trajSeqMIDDLE = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(8, 45))
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(8, 32))
                .waitSeconds(1.5)
                .forward(10)
                .lineToConstantHeading(new Vector2d(60, 60))
                .build();
        TrajectorySequence trajSeqRIGHT = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(22.5, 45))
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(22.5, 40))
                .waitSeconds(1.5)
                .forward(20)
                .lineToConstantHeading(new Vector2d(60, 60))
                .build();
        waitForStart();

        if (!isStopRequested() && LEFT)
            drive.followTrajectorySequence(trajSeqLEFT);
        if (!isStopRequested() && MIDDLE)
            drive.followTrajectorySequence(trajSeqMIDDLE);
        if (!isStopRequested() && RIGHT)
            drive.followTrajectorySequence(trajSeqRIGHT);
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
        telemetryTfod();
    }
    private void telemetryTfod() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x < 170) {
                LEFT = true;
                telemetry.addData("Left","");
            }
            if (x > 180 && x < 450) {
                MIDDLE = true;
                telemetry.addData("Middle","");
            }
            else {
                RIGHT = true;
                telemetry.addData("Right","");
            }
        }
    }
}
