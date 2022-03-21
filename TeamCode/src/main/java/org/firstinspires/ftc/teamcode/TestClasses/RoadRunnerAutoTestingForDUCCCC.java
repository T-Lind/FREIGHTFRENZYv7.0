package org.firstinspires.ftc.teamcode.TestClasses;

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

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "drive")
public class RoadRunnerAutoTestingForDUCCCC extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;


        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                .setAccelConstraint((a,e,c,d)->25)
                .splineTo(new Vector2d(-33, -21), Math.toRadians(90))

                .build();
        drive.followTrajectorySequence(traj1);
        drive.turn(Math.toRadians(90));



        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(-34, -21, Math.toRadians(-180)))//
                .setAccelConstraint((a,e,c,d)->35)
                .splineTo(new Vector2d(-63, -58.5), Math.toRadians(-90))

                .build();
        drive.followTrajectorySequence(traj2);

//spin duck

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(-63, -58.5, Math.toRadians(-90)))
                .strafeLeft(20)
                .turn(Math.toRadians(-20))
                .setAccelConstraint((a,e,c,d)->5)
                .lineTo(new Vector2d(-60,-61))

                .build();
        drive.followTrajectorySequence(traj3);

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .setReversed(true)
                .setAccelConstraint((a,e,c,d)->25)
                .splineTo(new Vector2d(-33, -26), Math.toRadians(0))

                .build();
        drive.followTrajectorySequence(traj4);
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())

                .lineTo(new Vector2d(-63, -38))

                .build();
        drive.followTrajectorySequence(traj5);























    }
}