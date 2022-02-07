package org.firstinspires.ftc.teamcode;

import android.graphics.PostProcessor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.PositionalDeviceTracker;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class RoadRunnerAutoTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(11,-63,Math.toRadians(90)))
                .splineTo(new Vector2d(1, -35.5), Math.toRadians(90))


                .build();
        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(-135));
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(1, -35.5, Math.toRadians(-45)))
                .splineTo(new Vector2d(55, -65), Math.toRadians(0))


                .build();
        drive.followTrajectory(traj2);
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(55, -65, Math.toRadians(0)))
                .setReversed(true)
                .back(45)
                .splineTo(new Vector2d(1, -35.5), Math.toRadians(135))


                .build();
        drive.followTrajectorySequence(traj3);











    }
}