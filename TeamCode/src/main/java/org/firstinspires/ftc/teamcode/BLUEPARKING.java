package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Disabled
@Autonomous(name="BLUEPARK", preselectTeleOp= "Beta_TeleOp")
public class BLUEPARKING extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(16.0, 62.5, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        waitForStart();

        TrajectorySequence trajSeqRIGHT = drive.trajectorySequenceBuilder(startPose)
                .waitSeconds(1.5)
                .lineToConstantHeading(new Vector2d(60, 60))
                .build();
        drive.followTrajectorySequence(trajSeqRIGHT);
    }
}
