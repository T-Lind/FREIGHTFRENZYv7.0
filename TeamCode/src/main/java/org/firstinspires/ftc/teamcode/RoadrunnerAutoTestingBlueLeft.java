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
public class RoadrunnerAutoTestingBlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(11, 63, Math.toRadians(-90)));

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, -63, Math.toRadians(90)))
                .splineTo(new Vector2d(10, 18), Math.toRadians(-90))
                .turn(Math.toRadians(90))


                .build();
        drive.followTrajectorySequence(traj1); //initial deposit

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(10, 18, Math.toRadians(0)))
                .lineTo(new Vector2d(0,53.6))
                // .lineTo(new Vector2d(58, -60))
                // .splineToConstantHeading(new Vector2d(2, -60), Math.toRadians(0))
                //.splineToConstantHeading(new Vector2d(2, -60), Math.toRadians(-180))
                .forward(65)


                .build();
        drive.followTrajectorySequence(traj2); //goes to warehouse


        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(58, 50, Math.toRadians(0)))
                //notice how y value on pose2d is -50 rather than -53.6, thats because strafing isn't and
                // wont ever be extremely accurate
                .setReversed(true)
                .back(60)
                .splineTo(new Vector2d(-12, 32), Math.toRadians(-90))


                .build();
        drive.followTrajectorySequence(traj3); //goes to deposit
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())//new Pose2d(-12, -42, Math.toRadians(90)))

                .splineTo(new Vector2d(1, -51), Math.toRadians(0))
                .forward(60)


                .build();

        drive.followTrajectorySequence(traj4); //goes back to warehouse, should have enough room to go forward
        // and accelerate over obstacle
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d(61, 51, Math.toRadians(0)))
                .setReversed(true)
                .back(60)
                .splineTo(new Vector2d(-12, 32), Math.toRadians(-90))


                .build();
        drive.followTrajectorySequence(traj5); //goes back to deposit
        //after repeating spline and going back and forth and whatnot,
        //  give a little bit room for error, we lose about 1-2 inches in localization in x and y



















    }
}