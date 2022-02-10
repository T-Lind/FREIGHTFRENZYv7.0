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

    private TrajectorySequence traj3;


    private double intakePower = -1;
    private int cycles = 0;

    private Rev2mDistanceSensor Distance;

    private double full = 0.0; //distance sensor reading for filled deposit
    private double reading;


    //IMPORTANT BOOLEANS FOR STATE MACHINES
    private boolean aman = true;
    private boolean runFetchFreight = false;
    private boolean runAutoCall = true;
    private boolean runDepositFreight = false;

    private ElapsedTime extend = new ElapsedTime();
    private ElapsedTime succ = new ElapsedTime();
    private ElapsedTime test = new ElapsedTime();


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




                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });


        starts();

        while (!opModeIsActive()) {
            //get the level, either 0, 1, or 2 or 3 (0 if not detected)
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
        //CHANGE THIS IN THE FUTURE -aman
        level = 3;
        telemetry.addData("DETECTED LEVEL: ",level);
        telemetry.update();

        camera.setPipeline(pipeline2);
        Distance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "detect");

        if(level == 1)
            liftTargetPos = 0;
         else if(level == 2)
            liftTargetPos = med;
        else
            liftTargetPos = top;






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


            if (level == 3) {
                v4b1.setPosition(.81);
                v4b2.setPosition(.81);
                telemetry.addData("V4b", v4b1.getPosition());
                telemetry.update();
            } else if(level == 1 || level == 2){
                v4b1.setPosition(.5);
                v4b2.setPosition(.5);
            }






                if (aman && !drive.isBusy()) {
                    dep.setPosition(.355);
                    test.reset();
                    sleep(300);
                    starts();
                    liftTargetPos = 0;
                    liftError = liftTargetPos - lift.getCurrentPosition();

                    lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));
                    liftB.setPower(lift.getPower());

                    aman = false;
                    reading = Distance.getDistance(DistanceUnit.MM);
                    runFetchFreight = true;
                    runDepositFreight = false;

                }

        }

    }


    public void safeGuard(){
        if(drive.getPoseEstimate().getY() < 50)
        intake.setPower(intakePower);
        intakeB.setPower(intakePower);

        reading = Distance.getDistance(DistanceUnit.MM);

        if(reading < 100){
            intakePower = 1;
            intake.setPower(intakePower);
            intakeB.setPower(intakePower);
            runDepositFreight = true;

        }

    }



    public void starts(){
        v4b1.setPosition(.19);
        v4b2.setPosition(.19);
        telemetry.addData("V4b", v4b1.getPosition());
        telemetry.update();
        dep.setPosition(.52);
        intake.setPower(0);
        intakeB.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();


        waitForStart();

        if (isStopRequested()) return;


        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));

        while(aman){
            redRight(); //Initial predeposit
            drive.update(); //Updates the drive asynch
            keepLiftAlive(); //Keeps the lift alive
        }

        starts();
        //The second while loop which will perform autocycling

        for(int i = 0; i < 2; i++){
            fetchFreight();
            liftTargetPos = 0;
            level = 0;

            while(drive.isBusy()){
                drive.update();
                safeGuard();
                keepLiftAlive();
            }

            liftTargetPos = top;
            level = 3;
            aman = true;

            depositCycledFreight();

            while(drive.isBusy()){
                drive.update();
                safeGuard();
                keepLiftAlive();
                intakePower = 0;
            }
            starts();
            intakePower = -1;
            cycles++;
        }


    }



    public void fetchFreight() throws InterruptedException {
        if(cycles == 0) {
            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(10, -20, Math.toRadians(0)))
                    .lineTo(new Vector2d(0, -53.6))
                    .forward(51.5)
                    .build();

            drive.followTrajectorySequenceAsync(traj2); //goes to warehouse
        } else{
            TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())//new Pose2d(-12, -42, Math.toRadians(90)))

                    .splineTo(new Vector2d(1, -51), Math.toRadians(0))
                    .forward(52)
                    .build();

            drive.followTrajectorySequenceAsync(traj4); //goes back to warehouse, should have enough room to go forward
            // and accelerate over obstacle
        }

    }



    public void depositCycledFreight() throws InterruptedException{
        if(cycles == 0) {
            traj3 = drive.trajectorySequenceBuilder(new Pose2d(49, -50, Math.toRadians(0)))
                    //notice how y value on pose2d is -50 rather than -53.6, thats because strafing isn't and
                    // wont ever be extremely accurate
                    .setReversed(true)
                    .back(55.5)
                    .splineTo(new Vector2d(-12, -33.5), Math.toRadians(90))


                    .build();
            drive.followTrajectorySequenceAsync(traj3); //goes to deposit
        } else{
            TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d(53, -51, Math.toRadians(0)))
                    .setReversed(true)
                    .back(56)
                    .splineTo(new Vector2d(-12, -33.5), Math.toRadians(90))


                    .build();
            drive.followTrajectorySequenceAsync(traj5); //goes back to deposit
        }
    }

    public void redRight() throws InterruptedException{

        if(runAutoCall) {

            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, -63, Math.toRadians(90)))
                    .splineTo(new Vector2d(10, -20), Math.toRadians(90))
                    .turn(Math.toRadians(-90))
                    .build();
            drive.followTrajectorySequenceAsync(traj1); //initial deposit

        }

        runAutoCall = false;

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




    }

}

