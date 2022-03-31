package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.CameraPipelines.CubeDetectionPipeline;
import org.firstinspires.ftc.teamcode.CameraPipelines.DuckDetectionPipeline;
import org.firstinspires.ftc.teamcode.CameraPipelines.TSEDetectionPipeline;
import org.firstinspires.ftc.teamcode.PIDS.LiftPID;
import org.firstinspires.ftc.teamcode.CameraPipelines.NewDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
public class Bot {
    private Telemetry telemetry;
    private Gamepad gamepad1,gamepad2;
    private LinearOpMode self;
    private TrajectorySequence trajectory;
    private RevColorSensorV3 color;
    private int depLevel = 0;
    private DcMotorEx intake, lift, ducc;
    private Servo arm1, arm2, dep, fold;
    private boolean delay = false;
    private LiftPID liftPID = new LiftPID(.0075, 0, .003);
    private int liftError = 0;
    private int liftTargetPos = 0;
    private WebcamName weCam;
    private OpenCvCamera camera;
    private TSEDetectionPipeline pipeline;
    private DuckDetectionPipeline pipeline2 = new DuckDetectionPipeline();
    private SampleMecanumDrive drive;
    Bot(LinearOpMode s, SampleMecanumDrive dr, Pose2d startingPos){
        self = s;
        telemetry = self.telemetry;
        gamepad1 = self.gamepad1;
        gamepad2 = self.gamepad2;
        HardwareMap hardwareMap = self.hardwareMap;
        drive = dr;
        drive.setPoseEstimate(startingPos);
        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        lift = (DcMotorEx) hardwareMap.dcMotor.get("LI");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setDirection(DcMotor.Direction.REVERSE);
        ducc = (DcMotorEx) hardwareMap.dcMotor.get("DU");
        ducc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ducc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ducc.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm1 = hardwareMap.servo.get("arm1");
        arm2 = hardwareMap.servo.get("arm2");
        dep = hardwareMap.servo.get("dep");
        fold = hardwareMap.servo.get("fold");
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        arm1.setDirection(Servo.Direction.REVERSE);
        weCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);
        pipeline = new TSEDetectionPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // @Override
            public void onOpened() {
                telemetry.update();
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        while (!self.opModeIsActive()) {
            depLevel = pipeline.getLevel();
            telemetry.addData("DETECTED LEVEL: ",depLevel);
            if(gamepad1.a)
                delay = true;
            telemetry.addData("Is delay turned on?", delay);
            telemetry.update();
        }
        // if the latest level was 0 then it must be in the 3 position.
        if(depLevel == 0)
            depLevel = 3;
        telemetry.addData("DETECTED LEVEL: ",depLevel);
        telemetry.update();
        camera.setPipeline(pipeline2);
        liftError = liftTargetPos - lift.getCurrentPosition();
        start();
    }
    public void updateLift(){
        liftError = liftTargetPos - lift.getCurrentPosition();
        lift.setPower(Range.clip(liftPID.getCorrection(liftError),-.7,1));
    }
    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!self.opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }

    public void start(){
        dep.setPosition(.45);
        fold.setPosition(.5);
        arm1.setPosition(.5);
        arm2.setPosition(.5);
        intake.setPower(0);
    }

    public Pose2d getStartingPos(){
        return drive.getPoseEstimate();
    }
    public int getDepLevel(){
        return depLevel;
    }
    public TrajectorySequence getCurrentTrajectory() { return trajectory; }
    public void setTrajectory(TrajectorySequence ts){
        trajectory=ts;
    }
    public void followTrajectory(){
        drive.followTrajectorySequenceAsync(trajectory);
        while(drive.isBusy()){
            drive.update();

            /*
            checkColorSensor();
            switch(intake)
                case 0: off
                case 1: intakeOn
                case 2: extake
            */
            updateLift();
        }
    }
    public void liftTo(int level){
        arm1.setPosition(.83);
        arm2.setPosition(.83);
        switch(level){
            case 1: liftTargetPos = 250;break;//bottom tier
            case 2: liftTargetPos = 500;break;//mid tier
            case 3: liftTargetPos = 1000;//top tier
        }
    }
    public void deposit(){
        dep.setPosition(.6);
    }
    public void liftDown(){
        arm1.setPosition(.5);
        arm2.setPosition(.5);
        liftTargetPos = 0;
    }
    public void spinDuck() throws InterruptedException {
        ElapsedTime spinTime = new ElapsedTime();
        ducc.setPower(-.3);
        while (spinTime.milliseconds() <= 3000) {
            heartbeat();
        }
        ducc.setPower(0);
    }
}