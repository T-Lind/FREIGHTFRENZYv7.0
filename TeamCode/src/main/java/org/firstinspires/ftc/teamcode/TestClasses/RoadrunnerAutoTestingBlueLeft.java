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
        double y = 39;
       // if(levels == 1){
          //  y += 1.5;
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, 63, Math.toRadians(-90)))
                .lineTo(new Vector2d(0, y))
                .turn(Math.toRadians(155))



                .build();
        drive.followTrajectorySequence(traj1); //initial deposit

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(-4, y, Math.toRadians(-205)))
                .setAccelConstraint((a,e,c,d)->30)
                .setVelConstraint((a,e,c,d)->40)
                .splineTo(new Vector2d(9,57), Math.toRadians(0))
                .forward(47)


                .build();
        drive.followTrajectorySequence(traj2); //goes to warehouse


        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(54, 57, Math.toRadians(0)))
                //notice how y value on pose2d is -50 rather than -53.6, thats because strafing isn't and
                // wont ever be extremely accurate
                .setReversed(true)
                .back(47)
                .setAccelConstraint((a,e,c,d)->30)
                .setVelConstraint((a,e,c,d)->40)
                .splineTo(new Vector2d(8, 37), Math.toRadians(-110)) //10
                .build();



        drive.followTrajectorySequence(traj3); //goes to deposit
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())//new Pose2d(-12, -42, Math.toRadians(90)))

                .splineTo(new Vector2d(8, 57), Math.toRadians(0))

                .forward(57)


                .build();

        drive.followTrajectorySequence(traj4); //goes back to warehouse, should have enough room to go forward
        // and accelerate over obstacle
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d(65, 55, Math.toRadians(0)))
                .setReversed(true)
                .back(59)


                .splineTo(new Vector2d(16, 37.5), Math.toRadians(-110))


                .build();
        drive.followTrajectorySequence(traj5); //goes back to deposit
        //after repeating spline and going back and forth and whatnot,
        //  give a little bit room for error, we lose about 1-2 inches in localization in x and y
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(new Pose2d(16, 38, Math.toRadians(70)))
                .splineTo(new Vector2d(9, 59), Math.toRadians(0))
                .forward(64)



                .build();
        drive.followTrajectorySequence(traj6);
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(new Pose2d(73, 59, Math.toRadians(0)))
                .back(54)
                .splineTo(new Vector2d(17, 41), Math.toRadians(-110))




                .build();
        drive.followTrajectorySequence(traj7);



















    }
}