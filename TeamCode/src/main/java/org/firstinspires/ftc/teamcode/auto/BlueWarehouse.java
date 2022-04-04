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



        bot.followTrajectory(drive.trajectorySequenceBuilder(new Pose2d(11,63, Math.toRadians(-90)))
                .addTemporalMarker(.05,() ->{
                    bot.liftTo(bot.getDepLevel());
                })
                .splineTo(new Vector2d(.5,34.5), Math.toRadians(-140))
                .build()
        );
        bot.depositAsync();

        for(int i = 0;i<3;i++) {
            bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                    .addTemporalMarker(.05, () -> {
                        bot.liftDown(); //Lift down
                    })
                    //Go to the freights
                    .setReversed(true)
                    /*.splineTo(new Vector2d(12.5, 64), Math.toRadians(0))
                    .strafeRight(3.5)*/
                    .splineToSplineHeading(new Pose2d(11,64.5,Math.toRadians(180)), Math.toRadians(60))
                    .setReversed(false)
                    .addTemporalMarker(2, () -> {
                        bot.setIntakeGo(true);
                    })
                    .back(35 + (i * 2))
                    //splineToConstantHeading(new Vector2d(46+(i*2), 64.5), Math.toRadians(180))
                    //splineToConstantHeading(new Vector2d(46+(i*2), 64.5-(i*.75)), Math.toRadians(180))
                    .build()
            );
            bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                    .addTemporalMarker(.75,() ->{
                        bot.setIntakeGo(false);
                    })
                    .addTemporalMarker(1, () -> {
                        bot.liftTo(3);

                    })
                    //splineToConstantHeading(new Vector2d(11, 64.5), Math.toRadians(180))
                    .forward(30 + (i * 2))
                    .splineTo(new Vector2d(-3.8, 41), Math.toRadians(-115))
                    .build()
            );
            bot.depositAsync();
        }

        //Park
        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
                .addTemporalMarker(.05, () -> {
                    bot.liftDown(); //Lift down
                })

                /*.addTemporalMarker(.2, () -> {
                    bot.keepLiftIntact();
                })*/

                //Go to the freights
                //.setReversed(true)
                //.splineToSplineHeading(new Pose2d(14, 70, Math.toRadians(180)), Math.toRadians(60))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(14,65,Math.toRadians(180)), Math.toRadians(60))
                .back(30)
                //.turn(Math.toRadians(-80))
                //.back(45)
            //    .setAccelConstraint((a,e,c,d)->55)
             //   .setVelConstraint((a,e,c,d)->80)

                .build()
        );


    }
}