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


@Autonomous(name = "BlueWarehouse")
public class BlueWarehouse extends LinearOpMode //creates class
{
    private Bot bot;
    private SampleMecanumDrive drive;
    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap),new Pose2d(11,63,Math.toRadians(-90)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        bot.setTrajectory(drive.trajectorySequenceBuilder(new Pose2d(11,63, Math.toRadians(-90)))
                .addTemporalMarker(.5,() ->{
                    bot.liftTo(bot.getDepLevel());
                })
                .addTemporalMarker(1.5,() ->{
                    bot.deposit();
                })
                .addTemporalMarker(2,() ->{
                    bot.liftDown();
                })
                .addTemporalMarker(9.5,() ->{
                    bot.liftTo(3);
                })
                .addTemporalMarker(9.8,() ->{
                    bot.deposit();
                })
                .splineTo(new Vector2d(2,36), Math.toRadians(-135))
                .setReversed(true)
                .splineTo(new Vector2d(14,64), Math.toRadians(0))
                .strafeRight(3.5)
                .setReversed(false)
                .back(35)
                .forward(30)
                .splineTo(new Vector2d(-1,45), Math.toRadians(-115))

                .build()
        );
        bot.followTrajectory();
    }
}