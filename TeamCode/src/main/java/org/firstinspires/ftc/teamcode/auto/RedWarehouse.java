package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;


@Autonomous(name = "RedWarehouse")
public class RedWarehouse extends LinearOpMode //creates class
{
    private Bot bot;
    private SampleMecanumDrive drive;
    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap), new Pose2d(11,-63,Math.toRadians(90)));
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initialize();
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getStartingPos())
                .build()
        );
    }
}


