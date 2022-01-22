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
import org.firstinspires.ftc.teamcode.CubeDetectionPipeline;
import org.firstinspires.ftc.teamcode.LiftPID;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


@Autonomous(name = "RedLeft")
public class RedLeft extends LinearOpMode //creates class
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
    private final int med = 275;


    private WebcamName weCam;
    private OpenCvCamera camera;
    private CubeDetectionPipeline pipeline;

    private SampleMecanumDrive drive; //d

    public void initialize() {

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


        weCam = hardwareMap.get(WebcamName.class, "Webcam 1");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);


        pipeline = new CubeDetectionPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // @Override
            public void onOpened() {
                telemetry.update();


                camera.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        Map<Integer,Double > levels = new HashMap();
        ElapsedTime eet = new ElapsedTime();
        while (!opModeIsActive()) {

            int bLevel = getLevel();

            levels.put(bLevel,eet.milliseconds());

            telemetry.addData("DETECTED LEVEL: ",bLevel);

            if(gamepad1.a)
                delay = true;


            telemetry.addData("Is delay turned on?", delay);
            telemetry.update();
        }

        double finalTime = eet.milliseconds();

        level = 3;
        liftTargetPos = top;

       if(levels.containsKey(1)){
           level = 1;
           liftTargetPos = 0;
       } else {
           for(double i : levels.keySet()){
               if(levels.get(i) >= finalTime - 2000 && i == 1) {
                   level = 2;
                   liftTargetPos = med;
               }
           }

       }


       // liftTargetPos = top;

        //liftTargetPos = top; //We might need to change this
        liftError = liftTargetPos - lift.getCurrentPosition();


    }

    // tiernan was here a lot
    public int getLevel() {

        int num = pipeline.getCubeNum();

        for (int i = 0; i < num; i++) {
            try {
                telemetry.addData("y pos", pipeline.getY(i));
                telemetry.addData("height", pipeline.getHeight(i));
                if ((pipeline.getHeight(i) > 80) && (pipeline.getY(i) < 400)) {
                    telemetry.addData("X Pos", pipeline.getX(i));
                    telemetry.addData("Y Pos", pipeline.getY(i));

                    if (pipeline.getX(i) > 150)
                        return 1;
                    else if ((pipeline.getX(i) < 150) && (pipeline.getX(i) > 0))
                        return 2;

                }
            }
            catch(Exception e){
                telemetry.addData("exception",0);
                telemetry.update();
            }
        }
        telemetry.update();
        return 3;
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
                    if (extend.milliseconds() < 1000) {
                        keepLiftAlive();

                        //Moves the virtual bars forward
                        v4b1.setPosition(targetV4B);
                        v4b2.setPosition(targetV4B);
                    }

                    if (extend.milliseconds() > 1000 && extend.milliseconds() < 2000) {
                        keepLiftAlive();

                        //Opens the deposit
                        dep.setPosition(.3);
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
        dep.setPosition(.52);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        if(delay){
            sleep(5000);
        }
        starts();
        redLeft();
        //liftAndDeposit();
    }

    public void redLeft() throws InterruptedException{
        waitForStart();

        if (isStopRequested()) return;
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(),true)
                .lineTo(new Vector2d(-17.85, -25.5))
                .build();
        drive.followTrajectory(traj3);
/*
        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .back(3)
                .build();

        drive.followTrajectory(traj3);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-3,0))
                .strafeRight(21.75)
                .build();

        drive.followTrajectory(traj);



        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-3,-21.75))
                .back(11.5)
                .build();

        drive.followTrajectory(traj2);
*/

        liftAndDeposit();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(-16.5,-25.5))
                .forward(11.5)
                .build();

        //DO NOT MESS WITH ANYTHING HERE AFTER
        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(-4.5,-24.5))
                .strafeLeft(48)
                .build();

        drive.followTrajectory(traj5);

        spinDuck();
        Trajectory traj9 = drive.trajectoryBuilder(new Pose2d(-4.5, 24.5))
                .back(2.5)
                .build();
        drive.followTrajectory(traj9);

        intake.setPower(-.55);
        intakeB.setPower(-.55);

       /*  Trajectory traj10 = drive.trajectoryBuilder(new Pose2d(-6.5, 23.5))
                .forward(6.5)
                .build();
        drive.followTrajectory(traj9);

       Trajectory traj = drive.trajectoryBuilder(new Pose2d(-4.5, 23.5))
                .forward(4.5)
                .build();
        drive.followTrajectory(traj);*/

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(-1.5, 23.5))
                .strafeRight(49.25)
                .build();
        drive.followTrajectory(traj6);
        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(-1.5, -25))
                .back(12)
                .build();
        drive.followTrajectory(traj7);
        intake.setPower(0);
        intakeB.setPower(0);
        liftTargetPos = top;
        liftAndDeposit();
        Trajectory traj8 = drive.trajectoryBuilder(new Pose2d(-13.5,-25),true)
                .lineTo(new Vector2d(-22.5, 25.75))
                .build();
        drive.followTrajectory(traj8);

     /*   Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(3.5,27.75))
                .back(29.5)
                .build();

        drive.followTrajectory(traj6);
        */
/*
        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(-14,-27))
                .strafeLeft(2)
                .build();

        drive.followTrajectory(traj7);*/

    }


    public void spinDuck() throws InterruptedException{
        ElapsedTime spinTime = new ElapsedTime();
        duccL.setPower(-.4);
        duccR.setPower(-.4);


        while (spinTime.milliseconds() <= 2000)
            heartbeat();


        duccL.setPower(-.2);
        duccR.setPower(-.2);


        while (spinTime.milliseconds() <= 4000)
           heartbeat();

        duccL.setPower(0);
        duccR.setPower(0);

     // duckIntake();


    }
    public void duckIntake() throws InterruptedException{

        ElapsedTime spinTime = new ElapsedTime();



        while (spinTime.milliseconds() <= 2000)
            heartbeat();
        intake.setPower(0);
        intakeB.setPower(0);
    }

}

