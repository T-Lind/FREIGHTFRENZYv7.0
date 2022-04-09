package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "RedWarehouse")
public class RedWarehouse extends LinearOpMode //creates class
{
    private Bot bot;
    private SampleMecanumDrive drive;
    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap),new Pose2d(11,-63,Math.toRadians(90)));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        bot.followTrajectory(drive.trajectorySequenceBuilder(new Pose2d(11,-63, Math.toRadians(90)))
                .addTemporalMarker(.05,() ->{
                    bot.liftTo(bot.getDepLevel());
                })
                .splineTo(new Vector2d(1,-35.53), Math.toRadians(145))
                .build()
        );
        bot.depositAsync();
        bot.cameraDeleter();
        //for(int i = 0;i<3;i++) {
        //cycle 1
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })
                //Go to the freights
                .setReversed(true)
                //.splineTo(new Vector2d(12.5, 64), Math.toRadians(0))
                //.strafeRight(3.5)
                .splineToSplineHeading(new Pose2d(9,-65.85,Math.toRadians(-180)), Math.toRadians(-60))
                .strafeLeft(1.4)
                .setReversed(false)
                .addTemporalMarker(2, () -> {
                    bot.setIntakeGo(true);
                })
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(44, 65), Math.toRadians(0))
                //.setAccelConstraint((a,e,c,d) -> 15)
                //.splineToConstantHeading(new Vector2d(45, 64), Math.toRadians(180))
                //.setAccelConstraint((a,e,c,d) -> 50)
                .back(35)
                .build()
        );
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.5,() ->{
                    bot.setIntakeGo(false);
                })
                .addTemporalMarker(1, () -> {
                    bot.liftTo(3);

                })
                //.setAccelConstraint((a,e,c,d) -> 43)
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(11, 65), Math.toRadians(180))
                .forward(28)
                //.setAccelConstraint((a,e,c,d) -> 43)
                .splineTo(new Vector2d(-5.25, -42), Math.toRadians(97)) //-3.8
                .build()
        );
        bot.depositAsync();
        //}
        //cycle 2
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })
                //Go to the freights
                .setReversed(true)
                //.splineTo(new Vector2d(12.5, 64), Math.toRadians(0))
                //.strafeRight(3.5)
                .splineToSplineHeading(new Pose2d(9,-66.2,Math.toRadians(-180)), Math.toRadians(-60))
                .strafeLeft(1.65)
                .setReversed(false)
                .addTemporalMarker(2, () -> {
                    bot.setIntakeGo(true);
                })
                .back(35)
                .lineTo(new Vector2d(47,-64))
                .lineTo(new Vector2d(46,-67.65))
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(45, 67), Math.toRadians(0))
                //.setAccelConstraint((a,e,c,d) -> 15)
                //.splineToConstantHeading(new Vector2d(47, 66), Math.toRadians(180))
                //.setAccelConstraint((a,e,c,d) -> 50)

                .build()
        );
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.5,() ->{
                    bot.setIntakeGo(false);
                })
                .addTemporalMarker(1, () -> {
                    bot.liftTo(3);

                })
                .setAccelConstraint((a,e,c,d) -> 43)
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(11, 67), Math.toRadians(180))
                .forward(32)
                //.setAccelConstraint((a,e,c,d) -> 43)
                .splineTo(new Vector2d(-4.5, -43.3), Math.toRadians(100))
                .build()
        );
        bot.depositAsync();
        //cycle 3
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })
                //Go to the freights
                .setReversed(true)
                //.splineTo(new Vector2d(12.5, 64), Math.toRadians(0))
                //.strafeRight(3.5)
                .splineToSplineHeading(new Pose2d(4,-65.7,Math.toRadians(-180)), Math.toRadians(-60))
                .strafeLeft(1.45)
                .setReversed(false)
                .addTemporalMarker(2, () -> {
                    bot.setIntakeGo(true);
                })
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(46, 68), Math.toRadians(0))
                //.setAccelConstraint((a,e,c,d) -> 15)
                //.splineToConstantHeading(new Vector2d(47, 67), Math.toRadians(180))
                //.setAccelConstraint((a,e,c,d) -> 50)
                .back(37)//40
                //Wiggle
                .lineTo(new Vector2d(52,-63.5))
                .lineTo(new Vector2d(41,-66.90))
                //.lineTo(new Vector2d(46,58))
                //.lineTo(new Vector2d(36,65))
                .build()
        );
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.5,() ->{
                    bot.setIntakeGo(false);
                })
                .addTemporalMarker(1, () -> {
                    bot.liftTo(3);

                })
                //.setAccelConstraint((a,e,c,d) -> 43)
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(11, 68), Math.toRadians(180))
                .forward(22) //35
                //.setAccelConstraint((a,e,c,d) -> 43)

                .splineTo(new Vector2d(-4.8, -44.95), Math.toRadians(101)) //-3.8,48, -106.5
                .build()
        );

        bot.depositAsync();

/*
        //Cycle 4
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })
                //Go to the freights
                .setReversed(true)
                //.splineTo(new Vector2d(12.5, 64), Math.toRadians(0))
                //.strafeRight(3.5)
                .splineToSplineHeading(new Pose2d(11,69,Math.toRadians(180)), Math.toRadians(60))
                .setReversed(false)
                .addTemporalMarker(2, () -> {
                    bot.setIntakeGo(true);
                })
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(46, 68), Math.toRadians(0))
                //.setAccelConstraint((a,e,c,d) -> 15)
                //.splineToConstantHeading(new Vector2d(47, 67), Math.toRadians(180))
                //.setAccelConstraint((a,e,c,d) -> 50)
                .back(43)
                //Wiggle
                //.lineTo(new Vector2d(46,58))
                //.lineTo(new Vector2d(36,65))
                .build()
        );
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.75,() ->{
                    bot.setIntakeGo(false);
                })
                .addTemporalMarker(1, () -> {
                    bot.liftTo(3);
                })
                //.setAccelConstraint((a,e,c,d) -> 43)
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(11, 68), Math.toRadians(180))
                .forward(38)
                //.setAccelConstraint((a,e,c,d) -> 43)
                .splineTo(new Vector2d(-3.8, 47), Math.toRadians(-105.5))
                .build()
        );
        bot.depositAsync();*/
        //Park
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })
                .setReversed(true)
                .setVelConstraint((a,e,c,d)->70)
                .setAccelConstraint((a,e,c,d)->90)
                .splineTo(new Vector2d(48,-68), Math.toRadians(0))

                .build()
        );
        bot.keepLiftIntact();


    }
}