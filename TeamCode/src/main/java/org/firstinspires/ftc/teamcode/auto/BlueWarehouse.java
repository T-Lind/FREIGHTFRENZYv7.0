package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


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

        bot.followTrajectory(drive.trajectorySequenceBuilder(new Pose2d(11,63, Math.toRadians(-90)))
                .addTemporalMarker(.05,() ->{
                    bot.liftTo(bot.getDepLevel());
                })
                .splineTo(new Vector2d(1,35), Math.toRadians(-145))
                .build()
        );
        bot.depositAsync();

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
                    .splineToSplineHeading(new Pose2d(11,65,Math.toRadians(180)), Math.toRadians(60))
                    .setReversed(false)
                    .addTemporalMarker(2, () -> {
                        bot.setIntakeGo(true);
                    })
                    //.setReversed(true)
                    //.splineToConstantHeading(new Vector2d(44, 65), Math.toRadians(0))
                    //.setAccelConstraint((a,e,c,d) -> 15)
                    //.splineToConstantHeading(new Vector2d(45, 64), Math.toRadians(180))
                    .back(35)
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
                    //.splineToConstantHeading(new Vector2d(11, 65), Math.toRadians(180))
                    .forward(30)
                    .splineTo(new Vector2d(-3.8, 42), Math.toRadians(-115))
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
                .splineToSplineHeading(new Pose2d(11,67,Math.toRadians(180)), Math.toRadians(60))
                .setReversed(false)
                .addTemporalMarker(2, () -> {
                    bot.setIntakeGo(true);
                })
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(45, 67), Math.toRadians(0))
                //.setAccelConstraint((a,e,c,d) -> 15)
                //.splineToConstantHeading(new Vector2d(47, 66), Math.toRadians(180))
                .back(37)
                .build()
        );
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.75,() ->{
                    bot.setIntakeGo(false);
                })
                .addTemporalMarker(1, () -> {
                    bot.liftTo(3);

                })
                .setAccelConstraint((a,e,c,d) -> 43)
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(11, 67), Math.toRadians(180))
                .forward(32)
                .splineTo(new Vector2d(-3.8, 44), Math.toRadians(-115))
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
                .splineToSplineHeading(new Pose2d(11,68,Math.toRadians(180)), Math.toRadians(60))
                .setReversed(false)
                .addTemporalMarker(2, () -> {
                    bot.setIntakeGo(true);
                })
                //.setReversed(true)
                //.splineToConstantHeading(new Vector2d(46, 68), Math.toRadians(0))
                //.setAccelConstraint((a,e,c,d) -> 15)
                //.splineToConstantHeading(new Vector2d(47, 67), Math.toRadians(180))
                .back(39)
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
                .forward(34)
                .splineTo(new Vector2d(-3.8, 46), Math.toRadians(-115))
                .build()
        );
        bot.depositAsync();
        //Park
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })
                .setReversed(true)
                .setAccelConstraint((a,e,c,d)->90)
                .splineTo(new Vector2d(55,68), Math.toRadians(0))

                .build()
        );


    }
}