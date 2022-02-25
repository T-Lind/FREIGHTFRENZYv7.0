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
public class RoadRunnerCycleTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, -63, Math.toRadians(90)))
                 .setAccelConstraint((a,e,c,d)->50)
                .splineTo(new Vector2d(10, -25), Math.toRadians(90))
                .turn(Math.toRadians(-90))
                //   .setReversed(true)
                // .splineTo(new Vector2d(-5, -33), Math.toRadians(120))



                .build();
        drive.followTrajectorySequence(traj1); //initial deposit

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(10, -25, Math.toRadians(0)))
                .setAccelConstraint((a,e,c,d)->50)
                .turn(Math.toRadians(-25))
                .lineTo(new Vector2d(9.5,-32))



                .build();
        drive.followTrajectorySequence(traj2); //goes to warehouse

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(), Math.toRadians(-25)))

                .forward(50)
                .build();
        drive.followTrajectorySequence(traj3);
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY(), Math.toRadians(-25)))
                .back(48)
                .build();
        drive.followTrajectorySequence(traj4);
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()+.5, Math.toRadians(-25)))
                .forward(51)
                .build();
        drive.followTrajectorySequence(traj5);
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()+.5, Math.toRadians(-25)))
                .back(49)
                .build();
        drive.followTrajectorySequence(traj6);
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX()+.25,drive.getPoseEstimate().getY()+.5, Math.toRadians(-25)))
                .forward(53)
                .build();
        drive.followTrajectorySequence(traj7);
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()+.5, Math.toRadians(-25)))
                .back(51)
                .build();
        drive.followTrajectorySequence(traj8);
        TrajectorySequence traj9 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()+.5, Math.toRadians(-25)))
                .forward(55)
                .build();
        drive.followTrajectorySequence(traj9);
        TrajectorySequence traj10 = drive.trajectorySequenceBuilder(new Pose2d(drive.getPoseEstimate().getX(),drive.getPoseEstimate().getY()+.5, Math.toRadians(-25)))
                .back(53)
                .build();
        drive.followTrajectorySequence(traj10);
        /*TrajectorySequence traj11 = drive.trajectorySequenceBuilder(traj10.end())
                .forward(52)
                .build();
        drive.followTrajectorySequence(traj11);
        TrajectorySequence traj12 = drive.trajectorySequenceBuilder(traj11.end())
                .back(50)
                .build();
        drive.followTrajectorySequence(traj12);
*/


























    }
}