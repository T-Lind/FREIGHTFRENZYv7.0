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
    private double targetV4B;
    private double depositTarget;
    private DcMotorEx lift, liftB , intake, intakeB;
    private Servo v4b1, v4b2, dep;
    private CRServo duccL, duccR;

    private TrajectorySequence traj3;
    private TrajectorySequence traj5;


    private double intakePower = -0.70;
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

    private final int top = 600;
    private final int med = 225;


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
        //dep.setPosition(.52);


        while (!opModeIsActive()) {
            //get the level, either 0, 1, or 2 or 3 (0 if not detected)
            level = pipeline.getLevel();
            telemetry.addData("DETECTED LEVEL: ",level);
            if(gamepad1.a)
                delay = true;


            telemetry.addData("Is delay turned on?", delay);
            telemetry.update();
        }


        targetV4B = 0;
        telemetry.addData("DETECTED LEVEL: ",level);
        telemetry.update();

        camera.setPipeline(pipeline2);
        Distance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "detect");

        if(level == 0)
            level = 3;
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

            if(level == 1) {
                targetV4B = .57;
                depositTarget = .36;

            }
            else if(level==2){
                targetV4B = .68;
                liftTargetPos=med;
                depositTarget = .3;
            }

            else if(level == 3) {
                targetV4B = .81;
                liftTargetPos=top;
                depositTarget = .36;
            }

            liftError = liftTargetPos - lift.getCurrentPosition();

            //Takes the lift up

            lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));
            liftB.setPower(lift.getPower());

            if(level != 0) {
                v4b1.setPosition(targetV4B);
                v4b2.setPosition(targetV4B);
            }

            if (aman && !drive.isBusy()) {
                    dep.setPosition(depositTarget);
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
            intake.setPower(intakePower);
            intakeB.setPower(intakePower);

            reading = Distance.getDistance(DistanceUnit.MM);

            if (reading < 100) {
                intakePower = 1.0;
                intake.setPower(intakePower);
                intakeB.setPower(intakePower);
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

        redRight(); //Initial predeposit

        while(aman){
            drive.update();
            keepLiftAlive();
        }

        starts();
        ElapsedTime whenToTurn = new ElapsedTime();
        int timer = 2750;

        for(int i = 0; i < 2; i++){
            fetchFreight();
            liftTargetPos = 0;
            level = 0;

             whenToTurn.reset();
            while(drive.isBusy()){
                drive.update();
                if(whenToTurn.milliseconds() > timer)
                    safeGuard();
                keepLiftAlive();

            }

            liftTargetPos = top;
            level = 3;
            aman = true;

            depositCycledFreight();

            ElapsedTime turnOffIntake = new ElapsedTime();
            while(drive.isBusy()){
                drive.update();
                keepLiftAlive();
                intakePower = 0;

                if(turnOffIntake.milliseconds() > 2250){
                    intake.setPower(0);
                    intakeB.setPower(0);
                }
            }
            starts();
            intakePower = -0.70;
            cycles++;

            timer = 1500;
        }

        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(traj5.end())
       .splineTo(new Vector2d(53, -50),Math.toRadians(0))

                .build();

        drive.followTrajectorySequenceAsync(traj6);



        liftTargetPos = 0;
        level = 0;
        aman = false;

        while(opModeIsActive()) {
            drive.update();
            keepLiftAlive();
        }

    }



    public void fetchFreight() throws InterruptedException {
        if(cycles == 0) {
            TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(10, -19, Math.toRadians(0)))
                    .lineTo(new Vector2d(0, -53.6))
                    .forward(52.5)
                    .build();

            drive.followTrajectorySequenceAsync(traj2); //goes to warehouse
        } else{
            TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())//new Pose2d(-12, -42, Math.toRadians(90)))

                    .splineTo(new Vector2d(1, -46), Math.toRadians(0))
                    .forward(59)
                    .build();

            drive.followTrajectorySequenceAsync(traj4); //goes back to warehouse, should have enough room to go forward
            // and accelerate over obstacle
        }

    }



    public void depositCycledFreight() throws InterruptedException{
        if(cycles == 0) {
            traj3 = drive.trajectorySequenceBuilder(new Pose2d(51, -50, Math.toRadians(0)))
                    //notice how y value on pose2d is -50 rather than -53.6, thats because strafing isn't and
                    // wont ever be extremely accurate
                    .setReversed(true)
                    .back(54)
                    .splineTo(new Vector2d(-15, -29.6), Math.toRadians(90))


                    .build();
            drive.followTrajectorySequenceAsync(traj3); //goes to deposit
        } else{
             traj5 = drive.trajectorySequenceBuilder(new Pose2d(58, -47, Math.toRadians(0)))
                    .setReversed(true)
                    .back(63)
                    .splineTo(new Vector2d(-15.7, -27.8), Math.toRadians(90))


                    .build();
            drive.followTrajectorySequenceAsync(traj5); //goes back to deposit
        }
    }

    public void redRight() throws InterruptedException{

            TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, -63, Math.toRadians(90)))
                    .splineTo(new Vector2d(8.5, -18), Math.toRadians(90))
                    .turn(Math.toRadians(-90))
                    .build();
            drive.followTrajectorySequenceAsync(traj1); //initial deposit
    }

}

