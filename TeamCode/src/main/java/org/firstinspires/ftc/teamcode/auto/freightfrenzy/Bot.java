package org.firstinspires.ftc.teamcode.auto.freightfrenzy;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Bot {
    private Telemetry telemetry;
    private LinearOpMode linearOpMode;
    private TrajectorySequence trajectory;
    private SampleMecanumDrive drive;

    public Bot(LinearOpMode s, SampleMecanumDrive dr, Pose2d startingPos){

        linearOpMode = s;
        telemetry = linearOpMode.telemetry;
        HardwareMap hardwareMap = linearOpMode.hardwareMap;
        drive = dr;
        drive.setPoseEstimate(startingPos);

        while (!linearOpMode.opModeIsActive()) {
            telemetry.addData("hi","");
        }
    }

    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!linearOpMode.opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }

    public TrajectorySequence getCurrentTrajectory() { return trajectory; }

    public void followTrajectory(TrajectorySequence ts) throws InterruptedException {
        trajectory = ts;
        drive.followTrajectorySequenceAsync(trajectory);
        while(drive.isBusy()){
            heartbeat();
            drive.update();

        }
    }

    public Pose2d getStartingPos(){
        return drive.getPoseEstimate();
    }
}