package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


@Autonomous(name = "BlueCarousel")
public class BlueCarousel extends LinearOpMode {
    private Bot bot;
    private SampleMecanumDrive drive;

    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap), new Pose2d(-36, 63, Math.toRadians(-90)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        bot.isDuck(true);
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getStartingPos())
                .addTemporalMarker(2, () -> {
                    bot.liftTo(bot.getDepLevel());
                })

                .splineTo(new Vector2d(-29.5, 25.5), Math.toRadians(0))

                .build()
        );

        bot.depositAsync();

        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(0.2, () -> {
                    bot.liftDown();
                })
                .setReversed(true)
                .splineTo(new Vector2d(-60, 59), Math.toRadians(-215))//go to ducc
                .setReversed(false)
                .build()
        );

        bot.spinDuck();

        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    bot.setIntakeGo(true);
                })
                .turn(Math.toRadians(-55))
                //.turn(Math.toRadians(-10))//get better angle to ducc
                // .turn(Math.toRadians(0))
                // .lineTo(new Vector2d(-40, -62))//strafe ducc
                .strafeLeft(20)
                .turn(Math.toRadians(17))
                // .setAccelConstraint((a,e,c,d)->7)
                .lineTo(new Vector2d(-60, 62))
                .turn(Math.toRadians(-17))
                .build()
        );

        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    bot.setIntakeGo(false);
                })
                .addTemporalMarker(2, () -> {
                    bot.liftTo(3);
                })
                .splineTo(new Vector2d(-33, 24), Math.toRadians(0))
                .build()
        );

        bot.depositAsync();

        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(0.2, () -> {
                    bot.liftDown();
                })
                .setReversed(true)
                .splineTo(new Vector2d(-65, 37), Math.toRadians(-180))
                .build()
        );


        /*bot.setTrajectory(drive.trajectorySequenceBuilder(bot.getTrajectory().end())
                .addTemporalMarker(0, () -> {
                    bot.turnOnIntake();
                })
                .turn(Math.toRadians(55))
                //.turn(Math.toRadians(-10))//get better angle to ducc
                // .turn(Math.toRadians(0))
                // .lineTo(new Vector2d(-40, -62))//strafe ducc
                .strafeRight(20)
                .turn(Math.toRadians(-17))
                // .setAccelConstraint((a,e,c,d)->7)
                .lineTo(new Vector2d(-60, -62))
                .turn(Math.toRadians(17))
                .splineTo(new Vector2d(-33, -24), Math.toRadians(0))
                .setReversed(true)
                .splineTo(new Vector2d(-65, -37), Math.toRadians(180))
                .build()
        );*/


    }
}