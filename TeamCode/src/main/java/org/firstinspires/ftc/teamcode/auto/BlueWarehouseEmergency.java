package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "BlueWarehouseEmergency")
public class BlueWarehouseEmergency extends LinearOpMode //creates class
{
    private Bot bot;
    private SampleMecanumDrive drive;
    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap),new Pose2d(11,63,Math.toRadians(-90)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        bot.followTrajectory(drive.trajectorySequenceBuilder(new Pose2d(11,63, Math.toRadians(-90)))
                .addTemporalMarker(.05,() ->{
                    bot.liftTo(bot.getDepLevel());
                    bot.missionFuckSebsAss();

                })
                .splineTo(new Vector2d(1,35.53), Math.toRadians(-145))
                .build()
        );
        bot.depositAsync();
        bot.cameraDeleter();

        //Park
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })
                .setReversed(true)
                .splineTo(new Vector2d(48,68), Math.toRadians(0))

                .build()
        );
        bot.keepLiftIntact();


    }
}