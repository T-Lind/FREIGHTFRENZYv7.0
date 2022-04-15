package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Autonomous (name = "BlueTest4x")
public class BlueTest4x extends LinearOpMode{
    private Bot bot;
    private SampleMecanumDrive drive;

    public void initialize() throws InterruptedException{
        bot = new Bot(this, drive = new SampleMecanumDrive(hardwareMap),new Pose2d(11,63,Math.toRadians(-90)));
    }

    @Override
    public void runOpMode() throws InterruptedException{
        initialize();
        bot.followTrajectory(
                drive.trajectorySequenceBuilder(new Pose2d(11,63, Math.toRadians(-90)))

                        //Initial deposit
                        .addTemporalMarker(0,() -> {
                            bot.liftTo(bot.getDepLevel());
                        })
                        .lineTo(new Vector2d(-11,42))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.deposit();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                            bot.liftDown();
                        })
                        .setReversed(true)
                        .UNSTABLE_addTemporalMarkerOffset(.5,()->{
                            bot.setIntakeGo(true);
                        })
                        .splineTo(new Vector2d(50,64.2), Math.toRadians(0))
                        .setReversed(false)
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.liftTo(3);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,()->{
                            bot.setIntakeGo(false);
                        })


                        //Cycle 1 deposit
                        .splineTo(new Vector2d(-11,42), Math.toRadians(270))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.deposit();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                            bot.liftDown();
                        })
                        .setReversed(true)
                        .UNSTABLE_addTemporalMarkerOffset(.5,()->{
                            bot.setIntakeGo(true);
                        })
                        .splineTo(new Vector2d(50,64.2), Math.toRadians(0))
                        .setReversed(false)

                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.liftTo(3);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,()->{
                            bot.setIntakeGo(false);
                        })


                        //Cycle 2 deposit
                        .splineTo(new Vector2d(-11,42), Math.toRadians(270))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.deposit();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                            bot.liftDown();
                        })
                        .setReversed(true)
                        .UNSTABLE_addTemporalMarkerOffset(.5,()->{
                            bot.setIntakeGo(true);
                        })
                        .splineTo(new Vector2d(50,64.2), Math.toRadians(0))
                        .setReversed(false)

                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.liftTo(3);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,()->{
                            bot.setIntakeGo(false);
                        })


                        //Cycle 3 deposit
                        .splineTo(new Vector2d(-11,42), Math.toRadians(270))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.deposit();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                            bot.liftDown();
                        })
                        .setReversed(true)
                        .UNSTABLE_addTemporalMarkerOffset(.5,()->{
                            bot.setIntakeGo(true);
                        })
                        .splineTo(new Vector2d(50,64.2), Math.toRadians(0))
                        .setReversed(false)
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.liftTo(3);
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,()->{
                            bot.setIntakeGo(false);
                        })


                        //Cycle 4 deposit
                        .splineTo(new Vector2d(-11,42), Math.toRadians(270))
                        .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                            bot.deposit();
                        })
                        .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                            bot.liftDown();
                        })
                        .setReversed(true)
                        .splineTo(new Vector2d(50,64.2), Math.toRadians(0))


                        .build()
        );
    }
}
