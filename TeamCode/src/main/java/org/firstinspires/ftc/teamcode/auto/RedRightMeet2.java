package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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


@Autonomous(name = "RedRightMeet2")
public class RedRightMeet2 extends LinearOpMode //creates class
{ //test test
    BNO055IMU imu;
    //test
    private int level = 0;

    private DcMotorEx lift, liftB, intake, intakeB;
    private Servo v4b1, v4b2, dep;
    private double targetDeposit;
    private CRServo duccL, duccR;
    private boolean delay = false;


    private ElapsedTime extend = new ElapsedTime();

    final int liftGrav = (int) (9.8 * 3);
    private LiftPID liftPID = new LiftPID(.05, 0, 0);
    private int liftError = 0;
    private int liftTargetPos = 0;

    private final int top = 620;
    private final int med = 130;


    private WebcamName weCam;
    private OpenCvCamera camera;
    private TSEDetectionPipeline pipeline;
    private DuckDetectionPipeline pipeline2 = new DuckDetectionPipeline();

    private SampleMecanumDrive drive; //d

    public void initialize() {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, -63, Math.toRadians(90)));

        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        lift = (DcMotorEx) hardwareMap.dcMotor.get("LI");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lift.setDirection(DcMotor.Direction.REVERSE);

        intakeB = (DcMotorEx) hardwareMap.dcMotor.get("INB");
        intakeB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftB = (DcMotorEx) hardwareMap.dcMotor.get("LIB");
        liftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftB.setDirection(DcMotor.Direction.REVERSE);

        v4b1 = hardwareMap.servo.get("v4b1");
        v4b2 = hardwareMap.servo.get("v4b2");
        dep = hardwareMap.servo.get("dep");
        duccL = hardwareMap.crservo.get("DL");
        duccR = hardwareMap.crservo.get("DR");

        drive = new SampleMecanumDrive(hardwareMap);


        duccL.setDirection(DcMotorSimple.Direction.FORWARD);

        v4b1.setDirection(Servo.Direction.REVERSE);


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

        while (!opModeIsActive()) {
            level = pipeline.getLevel();
            telemetry.addData("DETECTED LEVEL: ",level);

            if(gamepad1.a)
                delay = true;


            telemetry.addData("Is delay turned on?", delay);
            telemetry.update();
        }
        dep.setPosition(.63);
        // if the latest level was 0 then it must be in the 3 position.
        if(level == 0)
            level = 3;
        telemetry.addData("DETECTED LEVEL: ",level);
        telemetry.update();

        camera.setPipeline(pipeline2);
        liftError = liftTargetPos - lift.getCurrentPosition();


    }


    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }

    public void keepLiftAlive(){
        if(true) {
            liftError = liftTargetPos - lift.getCurrentPosition();

            //Takes the lift up

            lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));
            liftB.setPower(lift.getPower());
            telemetry.addData("Target Position", liftTargetPos);
            telemetry.addData("Current position", lift.getCurrentPosition());
            telemetry.update();
        }
    }


    public void liftAndDeposit() throws InterruptedException{
        double targetV4B = 0;
        telemetry.addLine("enteredLift");
        telemetry.update();
        if(level == 1) {
            targetV4B = .81;
            liftTargetPos=15;

            targetDeposit = .3;


        }
        else if(level==2){
            targetV4B=.7;
            liftTargetPos=med;
            targetDeposit = .3;
        }

        else if(level==3) {
            targetV4B = .81;
            liftTargetPos=top;
            targetDeposit = .3;
        }

        liftError = liftTargetPos - lift.getCurrentPosition();

        boolean depositRun = true;

        while(liftError > 50){
            liftError = liftTargetPos - lift.getCurrentPosition();

            //Takes the lift up
            lift.setPower(Range.clip(liftPID.getCorrection(liftError), -1, 1));
            liftB.setPower(lift.getPower());

        }

        extend.reset();

        while(depositRun){
            telemetry.addData("time", extend.milliseconds());
            telemetry.update();
            keepLiftAlive();
            //while(!die) {
            if (extend.milliseconds() < 1000) {
                keepLiftAlive();

                //Moves the virtual bars forward
                v4b1.setPosition(targetV4B);
                v4b2.setPosition(targetV4B);
            }

            if (extend.milliseconds() > 1000 && extend.milliseconds() < 2000) {
                keepLiftAlive();

                //Opens the deposit
                dep.setPosition(targetDeposit);
            }

            if (extend.milliseconds() > 2000 && extend.milliseconds() < 2500) {
                keepLiftAlive();
                dep.setPosition(.52);
                //Moves the virtual bars backward
                v4b1.setPosition(.19);
                v4b2.setPosition(.19);
            }
            if (extend.milliseconds() > 2500 && extend.milliseconds() < 3500) {

                //Gravity pulls the lift down
                liftTargetPos = 0;
                keepLiftAlive();



            }
            if(extend.milliseconds() > 3500){
                depositRun = false;
            }

            //    depositRun = false;
        }
        // }



    }


    public void starts(){
        v4b1.setPosition(.19);
        v4b2.setPosition(.19);
        dep.setPosition(.63);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        if(delay){
            sleep(5000);
        }
        starts();
        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));
        redRight();
        //liftAndDeposit();
    }

    public void redRight() throws InterruptedException {


        if (isStopRequested()) return;
        double y = -42.5;
        level =1;
        if (level == 1) {
            y = y - 2;
        }
        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, -63, Math.toRadians(90)))
                .setAccelConstraint((a, e, c, d) -> 35)
                .lineTo(new Vector2d(-2, y))


                .build();
        drive.followTrajectorySequence(traj1);
        drive.turn(Math.toRadians(-155));

        liftAndDeposit();
    }
}