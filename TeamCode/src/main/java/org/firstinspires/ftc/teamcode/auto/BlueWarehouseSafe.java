package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CameraPipelines.CubeDetectionPipeline;
import org.firstinspires.ftc.teamcode.CameraPipelines.DuckDetectionPipeline;
import org.firstinspires.ftc.teamcode.CameraPipelines.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.PIDS.LiftPID;
import org.firstinspires.ftc.teamcode.CameraPipelines.NewDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


@Autonomous(name = "BlueWarehouseSafe")
public class BlueWarehouseSafe extends LinearOpMode //creates class
{
    private Bot bot;
    private SampleMecanumDrive drive;
    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap),new Pose2d(11,63,Math.toRadians(-90)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        /*bot.followTrajectory(drive.trajectorySequenceBuilder(new Pose2d(11,63, Math.toRadians(-90)))
                .addTemporalMarker(.05,() ->{
                    bot.liftTo(bot.getDepLevel());
                })/*
                .addTemporalMarker(1.33,() ->{ //1.5 for top
                    bot.deposit();
                })
                .addTemporalMarker(2,() ->{
                    bot.liftDown();
                })
                .addTemporalMarker(6.5,() ->{
                    bot.liftTo(3);
                })
                .addTemporalMarker(7.5,() ->{
                    bot.deposit();
                })
                .addTemporalMarker(8,() ->{
                    bot.liftDown();
                })
                .addTemporalMarker(12.5,() ->{
                    bot.liftTo(3);
                })
                .addTemporalMarker(13.5,() ->{
                    bot.deposit();
                })
                .addTemporalMarker(14,() ->{
                    bot.liftDown();
                })*/
        //.splineTo(new Vector2d(1.15,35), Math.toRadians(-135))
                /*.addDisplacementMarker(() -> {
                    bot.deposit();
                })
                .setReversed(true)
                //GET THE FREIGHT PART 1
                .splineTo(new Vector2d(14,64), Math.toRadians(0))
                .strafeRight(3.5)
                .setReversed(false)
                .back(35)
                //DEPOSIT THE FREIGHT PART 1
                .forward(30)
                .splineTo(new Vector2d(-1,45), Math.toRadians(-115))
                .setReversed(true)
                //GET THE FREIGHT PART 2
                .splineTo(new Vector2d(14,64), Math.toRadians(0))
                .strafeRight(3.5)
                .setReversed(false)
                .back(35)
                //DEPOSIT THE FREIGHT PART 2
                .forward(30)
                .splineTo(new Vector2d(-1,45), Math.toRadians(-115))
                .setReversed(true)
                //GET THE FREIGHT PART 3
                .splineTo(new Vector2d(14,64), Math.toRadians(0))
                .strafeRight(3.5)
                .setReversed(false)
                .back(35)
                //DEPOSIT THE FREIGHT PART 3
                .forward(30)
                .splineTo(new Vector2d(-1,45), Math.toRadians(-115))
                .build()
        ); */

        //PRELOAD
        /*
        bot.followTrajectory(drive.trajectorySequenceBuilder(new Pose2d(11,63, Math.toRadians(-90)))
               .splineTo(new Vector2d(0, 36), Math.toRadians(-120))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(18.5, 65, Math.toRadians(180)), Math.toRadians(60))
                .setReversed(false)
                .back(35)
                .forward(30)
                .splineToConstantHeading(new Vector2d(18.5, 58), Math.toRadians(180))
                .splineTo(new Vector2d(-1, 45), Math.toRadians(245))
                .build()
        );
         */
        //bot.depositAsync();


        //initial
        bot.followTrajectory(drive.trajectorySequenceBuilder(new Pose2d(11,63, Math.toRadians(-90)))
                .addTemporalMarker(.05,() ->{
                    bot.liftTo(bot.getDepLevel());
                })
                .splineTo(new Vector2d(1.15,35.75), Math.toRadians(-135))
                .build()
        );
        bot.depositAsync();

        for(int i = 0;i<2;i++) {
            bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                    .addTemporalMarker(.05, () -> {
                        bot.liftDown(); //Lift down
                    })
                    //Go to the freights
                    .setReversed(true)
                    .splineTo(new Vector2d(14, 64), Math.toRadians(0))
                    .strafeRight(3.5+(i*.2))
                    .setReversed(false)
                    .addTemporalMarker(2, () -> {
                        bot.setIntakeGo(true);
                    })
                    .back(35 + (i * 1.75))
                    .build()
            );
            bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                    .addTemporalMarker(.75,() ->{
                        bot.setIntakeGo(false);
                    })
                    .addTemporalMarker(1, () -> {
                        bot.liftTo(3);

                    })
                    .forward(30 + (i * 1.75))
                    .splineTo(new Vector2d(-4.5, (43.75+(i*.2))), Math.toRadians(-100))
                    .build()
            );
            bot.depositAsync();
        }
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })
                //Go to the freights
                .setReversed(true)
                .splineTo(new Vector2d(14, 64), Math.toRadians(0))
                .strafeRight(3.5)
                .setReversed(false)
                .addTemporalMarker(2, () -> {
                    bot.setIntakeGo(true);
                })
                .back(27)
                .lineTo(new Vector2d(55,57))
                .build()
        );
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.75,() ->{
                    bot.setIntakeGo(false);
                })
                .addTemporalMarker(1, () -> {
                    bot.liftTo(3);

                })
                .lineTo(new Vector2d(52,65))
                .forward(30)
                .splineTo(new Vector2d(-4.5, 44), Math.toRadians(-100))
                .build()
        );
        bot.depositAsync();
        //Park
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })

                .addTemporalMarker(.2, () -> {
                    bot.keepLiftIntact();
                })

                //Go to the freights
                .setReversed(true)
                .setAccelConstraint((a,e,c,d)->90)
                .splineTo(new Vector2d(55,68), Math.toRadians(0))
                .build()
        );


    }
}