package org.firstinspires.ftc.teamcode.auto.roadrunner;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.SequenceWrapper;
import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.WrapperBuilder;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name="RoadrunnerCustomTest")
public class RoadrunnerCustomTest extends LinearOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        SequenceWrapper wrapper = new SequenceWrapper(
                new WrapperBuilder(drive, startPose)
                        .splineTo(20, 10, 90)
        );


        TrajectorySequence trajectorySequence = wrapper.trajectorySequenceBuilder.build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectorySequence(trajectorySequence);

//        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
//                .splineTo(new Vector2d(10, 0), 0)
//                .splineTo(new Vector2d(10,10),Math.toRadians(90))
//                .addTemporalMarker(1, () -> {
//                    telemetry.addData("","Ran!");
//                    telemetry.update();
//                })
//                .splineToLinearHeading(new Pose2d(10, 20), 0)
//                .back(10)
//                .turn(Math.toRadians(90))
//                .build();
//
//        waitForStart();
//
//        if(isStopRequested()) return;
//
//        drive.followTrajectorySequence(trajectorySequence);
    }
}
