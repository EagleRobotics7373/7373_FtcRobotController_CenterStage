package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;


@Autonomous(name="Nov 28", preselectTeleOp= "Beta_TeleOp")
public class TESTS extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(16.0, -62.0, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        waitForStart();


        TrajectorySequence trajSeqRIGHT = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(17, -40))
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(22.5, -38))
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(22.5, -45))
                .lineToConstantHeading(new Vector2d(40, -45))
                .build();
        drive.followTrajectorySequence(trajSeqRIGHT);

        targetFound = false;
        desiredTag  = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                } else {
                    telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                }
            } else {
                telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
            }
        }
        while (targetFound && opModeIsActive()) {
            if ((desiredTag.ftcPose.range - DESIRED_DISTANCE) > .1) {
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                aprilDrive = -Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                aprilTurn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                aprilStrafe = -Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
            }
            else {
                break;
            }

            moveRobot(aprilDrive, aprilStrafe, aprilTurn);
    }
        
}
