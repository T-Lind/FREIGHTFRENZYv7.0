package org.firstinspires.ftc.teamcode.TestClasses;

import android.graphics.PostProcessor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
//import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
//import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.PositionalDeviceTracker;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(group = "drive")
public class RoadRunnerCycleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(-4, -46), Math.toRadians(90))
                .turn(Math.toRadians(-155))
                .build();
        drive.followTrajectorySequence(traj1);
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(-4, -46, Math.toRadians(-65)))
                .splineTo(new Vector2d(20,-58), Math.toRadians(0))
                .forward(26)
                .build();
        drive.followTrajectorySequence(traj2);
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(46, -58, Math.toRadians(0)))
                .setReversed(true)
                .back(26)
                .splineTo(new Vector2d(-5,-43),Math.toRadians(110))
                .build();
        drive.followTrajectorySequence(traj3);
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d(-5, -43, Math.toRadians(-70)))
                .splineTo(new Vector2d(20,-58), Math.toRadians(0))
                .forward(28)
                .build();
        drive.followTrajectorySequence(traj4);
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d(48, -58, Math.toRadians(0)))
                .setReversed(true)
                .back(28)
                .splineTo(new Vector2d(-5,-43),Math.toRadians(110))
                .build();
        drive.followTrajectorySequence(traj5);
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(new Pose2d(-5, -43, Math.toRadians(-70)))
                .splineTo(new Vector2d(20,-58), Math.toRadians(0))
                .forward(30)
                .build();
        drive.followTrajectorySequence(traj6);
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(new Pose2d(50, -58, Math.toRadians(0)))
                .setReversed(true)
                .back(30)
                .splineTo(new Vector2d(-5,-43),Math.toRadians(110))
                .build();
        drive.followTrajectorySequence(traj7);
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(new Pose2d(-5, -43, Math.toRadians(-70)))
                .splineTo(new Vector2d(20,-58), Math.toRadians(0))
                .forward(32)
                .build();
        drive.followTrajectorySequence(traj8);
        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(new Pose2d(52, -58, Math.toRadians(0)))
                .setReversed(true)
                .back(32)
                .splineTo(new Vector2d(-5,-43),Math.toRadians(110))
                .build();
        drive.followTrajectorySequence(traj9);
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(new Pose2d(-5, -43, Math.toRadians(-70)))
                .splineTo(new Vector2d(20,-58), Math.toRadians(0))
                .forward(34)
                .build();
        drive.followTrajectorySequence(traj10);
        TrajectorySequence traj11 = drive.trajectorySequenceBuilder(new Pose2d(54, -58, Math.toRadians(0)))
                .setReversed(true)
                .back(34)
                .splineTo(new Vector2d(-5,-43),Math.toRadians(110))

                .build();
        drive.followTrajectorySequence(traj11);





































    }
}