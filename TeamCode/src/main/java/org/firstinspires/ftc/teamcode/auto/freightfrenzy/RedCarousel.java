//package org.firstinspires.ftc.teamcode.auto.freightfrenzy;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//
//
//@Autonomous(name = "Red Carousel")
//public class RedCarousel extends LinearOpMode {
//    private Bot bot;
//    private SampleMecanumDrive drive;
//
//    public void initialize() throws InterruptedException{
//        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap), new Pose2d(-36, -63, Math.toRadians(90)));
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        initialize();
//        bot.isDuck(true);
//        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getStartingPos())
//                .addTemporalMarker(.5, () -> {
//                    bot.liftTo(bot.getDepLevel());
//                })
//                .splineTo(new Vector2d(-25.5, -35.5), Math.toRadians(45))
//                .build()
//        );
//
//        bot.depositAsync();
//        bot.cameraDeleter();
//
//        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
//                .addTemporalMarker(0.2, () -> {
//                    bot.liftDown();
//                })
//                .setReversed(true)
//                .splineTo(new Vector2d(-59, -58.75), Math.toRadians(235))//go to ducc
//                .setReversed(false)
//                .build()
//        );
//
//        bot.spinDuck(true);
//
//        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    bot.setIntakeGo(true);
//                })
//                .turn(Math.toRadians(35))
//                //.turn(Math.toRadians(-10))//get better angle to ducc
//                // .turn(Math.toRadians(0))
//                // .lineTo(new Vector2d(-40, -62))//strafe ducc
//                .setAccelConstraint((a,e,c,d)->17)
//                .setVelConstraint((a,e,c,d)->25)
//
//                .strafeRight(25)
//                .turn(Math.toRadians(-21.8))
//                .setAccelConstraint((a,e,c,d)->7)
//                .setVelConstraint((a,e,c,d)->20)
//                .lineTo(new Vector2d(-60.5, -61.75))
//                .turn(Math.toRadians(21.8))
//
//                .setAccelConstraint((a,e,c,d)->25)
//                .setVelConstraint((a,e,c,d)->25)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    bot.setIntakeGo(false);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    bot.liftTo(3);
//                })
//                .splineTo(new Vector2d(-29.67, -25), Math.toRadians(0))//-15
//                .build()
//        );
//
//        bot.depositAsync();
//
//        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
//                .addTemporalMarker(0.2, () -> {
//                    bot.liftDown();
//                })
//                .setReversed(true)
//                .splineTo(new Vector2d(-65, -37.25), Math.toRadians(-180))
//                .build()
//        );
//      /*  initialize();
//        bot.isDuck(true);
//        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getStartingPos())
//                .addTemporalMarker(.5, () -> {
//                    bot.liftTo(bot.getDepLevel());
//                })
//                .splineTo(new Vector2d(-30.5, -25.5), Math.toRadians(0))
//                .build()
//        );
//
//        bot.depositAsync();
//
//        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
//                .addTemporalMarker(0.2, () -> {
//                    bot.liftDown();
//                })
//                .setReversed(true)
//                .splineTo(new Vector2d(-59.5, -59.8), Math.toRadians(235))//go to ducc
//                .setReversed(false)
//                .build()
//        );
//
//        bot.spinDuck(true);
//
//        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    bot.setIntakeGo(true);
//                })
//                .turn(Math.toRadians(35))
//                //.turn(Math.toRadians(-10))//get better angle to ducc
//                // .turn(Math.toRadians(0))
//                // .lineTo(new Vector2d(-40, -62))//strafe ducc
//                .setAccelConstraint((a,e,c,d)->7)
//                .setVelConstraint((a,e,c,d)->20)
//
//                .strafeRight(30)
//                .turn(Math.toRadians(-21.8))
//                .lineTo(new Vector2d(-24, -60.25))
//                .lineTo(new Vector2d(-60.5, -61.75))
//                .turn(Math.toRadians(21.8))
//
//                .setAccelConstraint((a,e,c,d)->25)
//                .setVelConstraint((a,e,c,d)->25)
//                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
//                    bot.setIntakeGo(false);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
//                    bot.liftTo(3);
//                })
//                .splineTo(new Vector2d(-32, -24), Math.toRadians(0))
//                .build()
//        );
//
//        bot.depositAsync();
//
//        bot.followTrajectory(drive.trajectorySequenceBuilder(bot.getCurrentTrajectory().end())
//                .addTemporalMarker(0.2, () -> {
//                    bot.liftDown();
//                })
//                .setReversed(true)
//                .splineTo(new Vector2d(-65, -37), Math.toRadians(180))
//                .build()
//        ); */
//    }
//}
//