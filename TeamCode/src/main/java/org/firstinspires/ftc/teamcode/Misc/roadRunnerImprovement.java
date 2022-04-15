/*package org.firstinspires.ftc.teamcode.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Point;

@Autonomous(name = "roadRunnerImprovement")
public class roadRunnerImprovemen extends LinearOpMode //creates class
{ //test test
    BNO055IMU imu;
    //test
    private int level = 0;

    private SampleMecanumDrive drive;
    public void initialize() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);

        while (!opModeIsActive()) {

        }
    }


    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        rri();
    }

    public void rri() throws InterruptedException{
        waitForStart();
        Pose2d botPos = new Pose2d();
        Trajectory traj1 = drive.trajectoryBuilder(botPos)
                .back(5)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(botPos)
                .strafeLeft(5)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(botPos)
                .forward(5)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(botPos)
                .splineTo(new Vector2d(10+botPos.getX(), 10+botPos.getY()), 90)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(botPos)
                .lineTo(new Vector2d(botPos.getX()+10,botPos.getY()+20))
                .build();


        drive.followTrajectory(traj1);
        botPos = drive.getPoseEstimate();
        drive.followTrajectory(traj2);
        botPos = drive.getPoseEstimate();
        drive.followTrajectory(traj3);
        botPos = drive.getPoseEstimate();
        drive.followTrajectory(traj4);
        botPos = drive.getPoseEstimate();
        drive.followTrajectory(traj5);

        if (isStopRequested()) return;




    }


}*/

