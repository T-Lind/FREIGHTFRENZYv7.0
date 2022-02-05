package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
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
import org.firstinspires.ftc.teamcode.CameraPipelines.CubeDetectionPipeline;
import org.firstinspires.ftc.teamcode.CameraPipelines.DuckDetectionPipeline;
import org.firstinspires.ftc.teamcode.PIDS.LiftPID;
import org.firstinspires.ftc.teamcode.CameraPipelines.NewDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


@Autonomous(name = "RedRight")
public class RedRight extends LinearOpMode //creates class
{ //test test
    BNO055IMU imu;

    private int level = 0;
    private boolean delay = false;
    private DcMotorEx lift, liftB , intake, intakeB;
    private Servo v4b1, v4b2, dep;
    private CRServo duccL, duccR;

    private int x = 0;
    private int y = 0;
    private int z = 0;

    private int cycles = 0;

    private Rev2mDistanceSensor Distance;

    private double full = 0.0; //distance sensor reading for filled deposit
    private double reading;


    //IMPORTANT BOOLEANS FOR STATE MACHINES
    private boolean aman = true;
    private boolean runAutoCycling = false;
    private boolean runAutoCall = true;
    private boolean runDepositFreight = false;

    private ElapsedTime extend = new ElapsedTime();
    private ElapsedTime succ = new ElapsedTime();


    final int liftGrav = (int) (9.8 * 3);
    private LiftPID liftPID = new LiftPID(.05, 0, 0);
    private int liftError = 0;
    private int liftTargetPos = 0;

    private final int top = 620;
    private final int med = 275;


    private WebcamName weCam;
    private OpenCvCamera camera;
    private NewDetectionPipeline pipeline;
    private DuckDetectionPipeline pipeline2 = new DuckDetectionPipeline();

    private SampleMecanumDrive drive;

    public void initialize() {

        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        lift = (DcMotorEx) hardwareMap.dcMotor.get("LI");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lift.setDirection(DcMotor.Direction.REVERSE);

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

        intakeB = (DcMotorEx) hardwareMap.dcMotor.get("INB");
        intakeB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        drive = new SampleMecanumDrive(hardwareMap);


        duccL.setDirection(DcMotorSimple.Direction.FORWARD);

        v4b1.setDirection(Servo.Direction.REVERSE);


        weCam = hardwareMap.get(WebcamName.class, "Webcam 1");


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());


        camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);


        pipeline = new NewDetectionPipeline();
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

        while (!opModeIsActive()) {
            //get the level, either 0, 1, or 2 (0 if not detected)
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
        Distance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "detect");





    }

    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }

    public void keepLiftAlive() throws InterruptedException{

        if(true) {

            liftError = liftTargetPos - lift.getCurrentPosition();

            //Takes the lift up

            lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));
            liftB.setPower(lift.getPower());

            if (liftTargetPos == 0 || level == 3) {
                v4b1.setPosition(.81);
                v4b2.setPosition(.81);
            } else{
                v4b1.setPosition(.5);
                v4b2.setPosition(.5);
            }


            if (aman && (!drive.isBusy())) {
                dep.setPosition(.355);
                sleep(450);
                starts();
                liftTargetPos = 0;
                liftError = liftTargetPos - lift.getCurrentPosition();

                lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));
                liftB.setPower(lift.getPower());

                aman = false;
                reading = Distance.getDistance(DistanceUnit.MM);
                runAutoCycling = true;
                runDepositFreight = false;

                cycles++;

            }
        }


    }



    public void starts(){
        v4b1.setPosition(.19);
        v4b2.setPosition(.19);
        dep.setPosition(.52);
        intake.setPower(0);
        intakeB.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();


        waitForStart();

        if (isStopRequested()) return;

        starts();

        redRight();

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .back(10)
                .turn(Math.toRadians(-90))
                .back(40)
                .build();

        //drive.followTrajectorySequence(trajSeq);

        ElapsedTime sherwin = new ElapsedTime();

        while(sherwin.milliseconds() < 10000){
            telemetry.addData("X", drive.getPoseEstimate().getX());
            telemetry.addData("Y", drive.getPoseEstimate().getY());
            telemetry.update();

        }


        /*
        while(aman && opModeIsActive()){
            redRight();
            drive.update();
            keepLiftAlive();
        }


        while(opModeIsActive()){
            fetchFreight();
            depositCycledFreight();

            if(!runDepositFreight)
            keepLiftAlive();

            drive.update();
            if(cycles == 2)
                break;
        }
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 3, Math.toRadians(90)))
                .strafeLeft(25)
                .forward(40)
                .build();

        drive.followTrajectorySequence(trajSeq);

    */

    }


    public void fetchFreight() throws InterruptedException {
        if (runAutoCycling) {
            //START: -35.5, 4
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-35.5, 3, Math.toRadians(90)))
                    .strafeLeft(36 + y)
                    .forward(25 + x)
                    .forward(12)
                    .build();
            //END: -9, 4
            drive.followTrajectorySequence(trajSeq);


            int i = 0;



            //sleep(300);
            succ.reset();
            while(succ.milliseconds() < 600){
                if (reading < full) {
                    intake.setPower(1);
                    intakeB.setPower(1);

                } else {
                    intake.setPower(-1);
                    intakeB.setPower(-1);

                }
            }

            intake.setPower(0);
            intakeB.setPower(0);



            runAutoCycling = false;
            runDepositFreight = true;
            x += 1.5;

        }
    }



    public void depositCycledFreight() throws InterruptedException{
        if(runDepositFreight){
            aman = true;

            liftTargetPos = top;

        if(z == 0) {
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-71.5 - y, 33.5, Math.toRadians(90)))
                    .back(9)
                    .back(23.5)
                    .strafeRight(29.5 + y)
                    .back(1.5)
                    .build();
            drive.followTrajectorySequenceAsync(trajSeq);

        } else{
            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(-71.5 - y, 33.5, Math.toRadians(90)))
                    .back(9)
                    .back(22.5 - z)
                    .strafeRight(29.5 + y)
                    .build();
            drive.followTrajectorySequenceAsync(trajSeq);

        }

        intake.setPower(0);
        intakeB.setPower(0);

            y++;
            z += 1.5;

            keepLiftAlive();

        }

        runDepositFreight = false;
    }

    public void redRight() throws InterruptedException{

                /*  CODE TO INTAKE DUCK - PLEASE READ THIS AND THE CODE
            FIRST, I TURN THE INTAKE ON.
            NEXT, I GET THE DISTANCE TO MOVE IN THE X AND Y DIRECTION
            NOTE THAT THE X AND Y DISTANCE TO MOVE IS RELATIVE TO THE ROBOT
            WHICH MEANS THAT THE X AND Y ARE SWITCHED IN ROADRUNNER.
            MAKE SURE TO COPY THE METHODS AT THE END OF THE PROGRAM!
            ALSO MAKE SURE THE PIPELINE SWITCH IS OCCURING AT THE END OF INIT.
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


        if(runAutoCall) {

            //.back means FORWARD (in direction of jerry)

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                  //  .back(27.5)
                    //.back(25)
                    //.turn(Math.toRadians(67.5))
                    //.back(6)
                    //.forward(6)
                    //.forward(54)
                    //.waitSeconds(3)
                    //.back(19)

                    //.back(41)
                    //.back(27.5)
                    //.turn(Math.toRadians(67.5))
                    //.back(6)

                    .splineTo(new Vector2d(6, 47), Math.toRadians(0))
                    //.setReversed(true)
                    //.splineTo(new Vector2d(-20, -14), Math.toRadians(180))
                    .build();


            //-35.5, 4

            drive.followTrajectorySequence(trajSeq);
            runAutoCall = false;
            level = 0;


        }

    }

}

