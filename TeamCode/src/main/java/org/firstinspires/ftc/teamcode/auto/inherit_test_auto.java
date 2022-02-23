package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CameraPipelines.DuckDetectionPipeline;
import org.firstinspires.ftc.teamcode.CameraPipelines.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.PIDS.LiftPID;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "inherit_test_auto")
public class inherit_test_auto extends inheritance //creates class
{
    // BUILD ROADRUNNER TRAJECTORIES
    TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, 63, Math.toRadians(-90)))
            .splineTo(new Vector2d(8.5, 16.75), Math.toRadians(-90))
            .turn(Math.toRadians(90))
            .build();


    // INITIALIZE AUTONOMOUS PROGRAM - INIT PHASE
    public void initialize() {
        int cnt = 0;
        while (!opModeIsActive()) {
            cnt++;
            telemetry.addData("cnt",cnt);
            telemetry.update();
            if(cnt>100)
                cnt=0;
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
        waitForStart();
        if (isStopRequested()) return;

        sequence();

        while(opModeIsActive()) {
            drive.update();
        }

    }
    public void sequence() throws InterruptedException{
        // EXECUTE MOVEMENTS
        drive.followTrajectorySequenceAsync(traj1);

        m1.setPower(0.50);

        resetTime();
        while(getTime() < 5000)
            m1.setPower(m1.getPower()-0.01);
        m1.setPower(0);
    }

}

