package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
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
import org.firstinspires.ftc.teamcode.CameraPipelines.DuckDetectionPipeline;
import org.firstinspires.ftc.teamcode.PIDS.LiftPID;
import org.firstinspires.ftc.teamcode.CameraPipelines.NewDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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


    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    private ElapsedTime extend = new ElapsedTime();

    final int liftGrav = (int) (9.8 * 3);
    private LiftPID liftPID = new LiftPID(.05, 0, 0);
    private int liftError = 0;
    private int liftTargetPos = 0;

    private final int top = 650;
    private final int med = 225;


    private WebcamName weCam;
    private OpenCvCamera camera;
    private NewDetectionPipeline pipeline;
    private DuckDetectionPipeline pipeline2 = new DuckDetectionPipeline();

    private SampleMecanumDrive drive;

    public void initialize() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(-90)));

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

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "lights");

        weCam = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);

        pipeline = new NewDetectionPipeline();
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
            //get the level, either 0, 1, or 2 (0 if not detected) - TO USE CHANGE PIPELINE
            level = pipeline.getLevel();
            telemetry.addData("DETECTED LEVEL: ",level);

            if(gamepad1.a)
                delay = true;


            telemetry.addData("Is delay turned on?", delay);
            telemetry.update();
        }

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
            targetV4B = .49;

        }
        else if(level==2){
            targetV4B=.65;
            liftTargetPos=med;
        }

        else if(level==3) {
            targetV4B = .81;
            liftTargetPos=top;
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
        }
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
    }

    public void blueRight() throws InterruptedException{
        waitForStart();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(-90)));
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-36, 63, Math.toRadians(-90)))
                .splineTo(new Vector2d(-32.5, 21), Math.toRadians(-90))

                .build();
        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(-90));

        liftAndDeposit();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-33, 21, Math.toRadians(-180)))//
                .splineTo(new Vector2d(-66, 61), Math.toRadians(-180))


                .build();
        drive.followTrajectory(traj2);

        spinDuck();
/*
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(-64, 61, Math.toRadians(180)),true)
                .splineTo(new Vector2d(-55, 35),Math.toRadians(270))

                .build();
        drive.followTrajectory(traj3);


        // code to intake duck - dx and dy might need to be swapped
        // cause based on what I know, dx and dy should be right but in my testing it was not.
        // also dy should be the right amount but we'll see, that's easy to adjust
       // intake.setPower(-.7);
       // intakeB.setPower(-.7);


       // double dx = pipeline2.getDucc_x();
       /* for(int i = 0;i<10;i++){
            double dx_add = pipeline2.getDucc_x();
            if(dx_add != Integer.MIN_VALUE)
                dx+=dx_add;
        }*/
       // dx/=-10;

       // double dy = 29;

        //telemetry.addData("dx: ",dx);
        //telemetry.update();

       /* Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-55, 40, Math.toRadians(90)))
                .lineTo(new Vector2d(-49-dx, 35+dy))

                .build();

        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-49+dx, 35+dy, Math.toRadians(90)),true)
                .splineTo(new Vector2d(-31,24),Math.toRadians(0))

                .build();
        drive.followTrajectory(traj5);

        intake.setPower(0);
        intakeB.setPower(0);

        liftAndDeposit();

        // I turn off intake here instead of earlier so that if the duck gets yeeted there is a possibility this yoinks it up
*/
        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-66, 60, Math.toRadians(-180)))
                .lineTo(new Vector2d(-67,39))

                .build();
        drive.followTrajectory(traj6);

        /*  CODE TO INTAKE DUCK - PLEASE READ THIS AND THE CODE
            FIRST, I TURN THE INTAKE ON.
            NEXT, I GET THE DISTANCE TO MOVE IN THE X AND Y DIRECTION
            NOTE THAT THE X AND Y DISTANCE TO MOVE IS RELATIVE TO THE ROBOT
            WHICH MEANS THAT THE X AND Y ARE SWITCHED IN ROADRUNNER.
            MAKE SURE TO COPY THE METHODS AT THE END OF THE PROGRAM!
            ALSO MAKE SURE THE PIPELINE SWITCH IS OCCURING AT THE END OF INIT.
            AS WELL AS THAT THE PIPELINE2 OBJECT IS CREATED AT START OF THE PROGRAM.
            MOVE THIS CODE WHERE YOU'D LIKE.
                GOOD LUCK - TIERNAN

        intake.setPower(-.75);
        intakeB.setPower(-.75);


        double dx = duccAttack();
        double dy = pipeline2.getDucc_y();

        Trajectory duccTraj = drive.trajectoryBuilder(new Pose2d(),true)
                .lineTo(new Vector2d(dy, dx))
                .build();

        drive.followTrajectory(duccTraj);

        intake.setPower(0);
        intakeB.setPower(0);*/
/*


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

        drive.followTrajectory(traj7);*/

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

    public static double get_dist(DuckDetectionPipeline pipeline){
        int cnt = 0;
        double mean = 0;
        while(10 > cnt){
            double dist_x = pipeline.getDucc_x();
            if(dist_x != Integer.MIN_VALUE) {
                cnt++;
                mean += dist_x;
            }
        }
        mean /= cnt;
        return mean;
    }

    public double duccAttack(){
        double dx = get_dist(pipeline2);
        while(dx < -100) {
            dx = get_dist(pipeline2);//pipeline2.getDucc_x();
            telemetry.addData("DX: ,", dx);
            telemetry.update();
        }
        return dx;
    }
}

