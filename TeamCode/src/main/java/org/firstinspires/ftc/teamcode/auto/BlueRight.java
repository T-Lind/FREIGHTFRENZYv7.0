package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.DuckDetectionPipeline;
import org.firstinspires.ftc.teamcode.LiftPID;
import org.firstinspires.ftc.teamcode.NewDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.imagePipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;



@Autonomous(name = "BlueRight")
public class BlueRight extends LinearOpMode //creates class
{ //test test
    BNO055IMU imu;

    private int level = 0;

    private DcMotorEx lift, liftB, intake, intakeB;
    private Servo v4b1, v4b2, dep;
    private CRServo duccL, duccR;
    private boolean delay = false;


    private ElapsedTime extend = new ElapsedTime();

    final int liftGrav = (int) (9.8 * 3);
    private LiftPID liftPID = new LiftPID(.05, 0, 0);
    private int liftError = 0;
    private int liftTargetPos = 0;

    private final int top = 650;
    private final int med = 350;


    private WebcamName weCam;
    private OpenCvCamera camera;
    private DuckDetectionPipeline pipeline;

    private SampleMecanumDrive drive;

    public void initialize() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);


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


        weCam = hardwareMap.get(WebcamName.class, "Webcam 2");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);


        //pipeline = new CubeDetectionPipeline();
        pipeline = new DuckDetectionPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // @Override
            public void onOpened() {
                telemetry.update();


                camera.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {

            }
        });



        while (!opModeIsActive()) {
            double angle = pipeline.getAngle();
            double distance = pipeline.getDistance();

            telemetry.addData("distance",distance);
            telemetry.addData("angle: ",angle);

            //get the level, either 0, 1, or 2 (0 if not detected) - TO USE CHANGE PIPELINE
            /*level = pipeline.getLevel();
            telemetry.addData("DETECTED LEVEL: ",level);

            if(gamepad1.a)
                delay = true;


            telemetry.addData("Is delay turned on?", delay);*/
            telemetry.update();
        }

        // if the latest level was 0 then it must be in the 3 position.
        if(level == 0)
            level = 3;
        telemetry.addData("DETECTED LEVEL: ",level);
        telemetry.update();
        ElapsedTime delaytime = new ElapsedTime();

        // JUST FOR TESTING - REMOVE FOR AUTO TO WORK!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        while (delaytime.milliseconds() <= 3000)
            heartbeat();
        //stop();


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
        if(level == 1)
            targetV4B = .5;
        else
            targetV4B = .81;

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
            if (extend.milliseconds() < 2000) {
                keepLiftAlive();

                //Moves the virtual bars forward
                v4b1.setPosition(targetV4B);
                v4b2.setPosition(targetV4B);
            }

            if (extend.milliseconds() > 2000 && extend.milliseconds() < 3000) {
                keepLiftAlive();

                //Opens the deposit
                dep.setPosition(.3);
            }
            if (extend.milliseconds() > 3000 && extend.milliseconds() < 4000) {
                keepLiftAlive();

                //Closes the deposit
                // keepLiftAlive();

                dep.setPosition(.52);
            }
            if (extend.milliseconds() > 4000 && extend.milliseconds() < 5000) {
                keepLiftAlive();

                //Moves the virtual bars backward
                v4b1.setPosition(.19);
                v4b2.setPosition(.19);
            }
            if (extend.milliseconds() > 5000 && extend.milliseconds() < 6500) {

                //Gravity pulls the lift down
                liftTargetPos = 0;
                keepLiftAlive();



            }
            if(extend.milliseconds() > 6500){
                depositRun = false;
            }

            //    depositRun = false;
        }
        // }



    }


    public void starts(){
        v4b1.setPosition(.19);
        v4b2.setPosition(.19);
        dep.setPosition(.52);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        if(delay){
            sleep(5000);
        }
        starts();
        blueRight();
        //liftAndDeposit();
    }

    public void blueRight() throws InterruptedException{
        waitForStart();

        if (isStopRequested()) return;

        double dx = pipeline.getDucc_x();
        double dy = pipeline.getDucc_y();

        intake.setPower(-.55);
        intakeB.setPower(-.55);

        Trajectory duccTraj = drive.trajectoryBuilder(new Pose2d(),true)
                .lineTo(new Vector2d(32, dx))
                .build();

        drive.followTrajectory(duccTraj);

        Trajectory duccTraj2 = drive.trajectoryBuilder(new Pose2d(),true)
                .strafeLeft(3)
                .build();

        drive.followTrajectory(duccTraj2);

        intake.setPower(0);
        intakeB.setPower(0);
        // REMOVE FOR FULL AUTO - DUCK TESTING
        stop();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .back(3)
                .build();

        drive.followTrajectory(traj3);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-3,0))
                .strafeLeft(21)
                .build();

        drive.followTrajectory(traj);



        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-3, 21))
                .back(16)
                .build();

        drive.followTrajectory(traj2);

        liftAndDeposit();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-18.5,21))
                .forward(14)
                .build();

        //DO NOT MESS WITH ANYTHING HERE AFTER
        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-4,21))
                .strafeRight(48.5)
                .build();

        drive.followTrajectory(traj5);

        spinDuck();

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-7,-27.5))
                .back(17)
                .build();

        drive.followTrajectory(traj6);

        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(-24,-27))
                .strafeRight(2)
                .build();

        drive.followTrajectory(traj7);

    }


    public void spinDuck() throws InterruptedException{
        ElapsedTime spinTime = new ElapsedTime();
        duccL.setPower(0.2);
        duccR.setPower(0.2);
        while (spinTime.milliseconds() <= 6000)
            heartbeat();
        duccL.setPower(0);
        duccR.setPower(0);

    }
}

