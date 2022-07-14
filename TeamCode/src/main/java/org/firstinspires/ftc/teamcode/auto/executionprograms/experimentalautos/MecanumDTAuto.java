package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.freightfrenzy.Bot;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;


@Autonomous(name = "MecanumDTAuto")
public class MecanumDTAuto extends LinearOpMode {
    private Bot bot;
    private SampleMecanumDrive drive;


    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap), new Pose2d(-36, 63, Math.toRadians(-90)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getStartingPos())
                .lineTo(new Vector2d(-35, 63))
                .build()
        );

    }
}
