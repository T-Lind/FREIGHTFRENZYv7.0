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

        waitForStart();

        if (isStopRequested()) return;

        Trajectory trajSeq = drive.trajectoryBuilder(new Pose2d())

                //.splineToConstantHeading(new Vector2d(-30, 6), Math.toRadians(0))
               .splineToLinearHeading(new Pose2d(-18.6, 18), Math.toRadians(0))

                //.splineTo(new Vector2d(6, 47), Math.toRadians(90))
                //.back(43) //"forward"
                //.turn(90)
                //.lineTo(new Vector2d(-17.85, -25.5))
                //.strafeLeft(20)
                /*.waitSeconds(1)
                .forward(16) //"back"
                .turn(Math.toRadians(70))
                .turn(Math.toRadians(-70))

                .splineTo(new Vector2d(6, 47), Math.toRadians(90))
                .waitSeconds(1)
                //.back(50)
                //.turn(Math.toRadians(-90))
                //.back(49)
                //.turn(Math.toRadians(-90))
                //.back(21)
                .setReversed(true)
                .splineTo(new Vector2d(-20, -14), Math.toRadians(180)) //reverse spline
                .turn(Math.toRadians(70))
                .turn(Math.toRadians(-70))
                .setReversed(false)
                .splineTo(new Vector2d(6, 51), Math.toRadians(90))*/
                .build();




        drive.followTrajectory(trajSeq);
        Trajectory trajSeq1 = drive.trajectoryBuilder(new Pose2d(-18.6,18))


                .splineTo(new Vector2d(35, 22), Math.toRadians(90))
                .build();
        drive.followTrajectory(trajSeq1);

        /*Trajectory traj3 = test.trajectoryBuilder(new Pose2d())
                .strafeRight(15)
                .build();*/

    }
}