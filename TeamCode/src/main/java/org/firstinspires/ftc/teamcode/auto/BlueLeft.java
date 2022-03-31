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


@Autonomous(name = "BlueLeft")
public class BlueLeft extends LinearOpMode //creates class
{
    private Bot bot;
    private SampleMecanumDrive drive;
    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap),new Pose2d(0,0,Math.toRadians(90)));
    }

    @Override
    public void runOpMode() {
        bot.start();
        bot.setTrajectory(
                drive.trajectorySequenceBuilder(bot.getStartingPos())
                .setVelConstraint((a,e,c,d)->20)
                .addTemporalMarker(2,()->{
                    bot.liftTo(3);
                })
                .addTemporalMarker(4,()->{
                    bot.liftDown();
                })
                .splineTo(new Vector2d(-23,23),Math.toRadians(180))//Part 1
                .splineTo(new Vector2d(-46,0),Math.toRadians(270))//Part 2
                .splineTo(new Vector2d(-23,-23),Math.toRadians(0))//Part 3
                .splineTo(new Vector2d(0,0),Math.toRadians(90))//Part 4

                .build()
        );
        bot.followTrajectory();
    }
}

