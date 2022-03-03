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
import org.firstinspires.ftc.teamcode.CameraPipelines.TSEDetectionPipeline;
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
    //test
    private int level = 0;
    private boolean delay = false;

    private DcMotorEx lift, liftB , intake, intakeB;
    private Servo v4b1, v4b2, dep;
    private CRServo duccL, duccR;


    private ArrayList<TrajectorySequence> trajectories;
    private double[] y_values;
    private double[] x_values;

    private double intakePower = -1;

    private Rev2mDistanceSensor Distance;

    private double reading;


    private boolean aman = true;



    final int liftGrav = (int) (9.8 * 3);
    private LiftPID liftPID = new LiftPID(.05, 0, 0);
    private int liftError = 0;
    private int liftTargetPos = 0;

    private final int top = 600;
    private final int med = 130;


    private WebcamName weCam;
    private OpenCvCamera camera;
    private TSEDetectionPipeline pipeline;
    private DuckDetectionPipeline pipeline2 = new DuckDetectionPipeline();

    private SampleMecanumDrive drive;

    public void initialize() throws InterruptedException {

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


        starts();

        initializeTrajectories();

        while (!opModeIsActive()) {
            //get the level, either 0, 1, or 2 or 3 (0 if not detected)
            level = pipeline.getLevel();
            telemetry.addData("DETECTED LEVEL: ",level);
            if(gamepad1.a)
                delay = true;
            telemetry.addData("Is delay turned on?", delay);
            telemetry.update();
        }


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

    public void initializeTrajectories() throws InterruptedException{
        trajectories = new ArrayList<TrajectorySequence>();
        y_values = new double[7];
        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));
        //Preload (0)
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(0, -40))
                .turn(Math.toRadians(-155))
                .build();

        //Going back to the warehouse, first cycle (1)
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(0, -40, Math.toRadians(-65)))
                .splineTo(new Vector2d(9,-57), Math.toRadians(0))
                .forward(45)
                .build();


        //Going to deposit freight, first cycle (2)
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(53, -57, Math.toRadians(0)))
                .setReversed(true)
                .back(57)
                .splineTo(new Vector2d(3, -35), Math.toRadians(110))
                .build();


        //Going back to warehouse, second cycle (3)
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d(2, -35, Math.toRadians(-70)))//new Pose2d(-12, -42, Math.toRadians(90)))
                .splineTo(new Vector2d(8, -62.5), Math.toRadians(0))
                .forward(60)
                .build();

        //Going to deposit freight, second cycle (4)
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d(68, -62.5, Math.toRadians(0)))
                .setReversed(true)
                .back(63)
                .splineTo(new Vector2d(10, -34), Math.toRadians(110))
                .build();


        //Going to warehouse, third cycle (5)
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(new Pose2d(10, -34, Math.toRadians(-70)))
                .splineTo(new Vector2d(9, -63), Math.toRadians(0))
                .forward(65)
                .build();

        //Going to deposit freight, third cycle (6)
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(new Pose2d(74, -63, Math.toRadians(0)))
                .back(62)
                .splineTo(new Vector2d(14, -33), Math.toRadians(110))
                .build();
        TrajectorySequence traj8 = drive.trajectorySequenceBuilder(traj7.end())

                .splineTo(new Vector2d(74,-59), Math.toRadians(0))
                .build();



        trajectories.add(traj1);
        trajectories.add(traj2);
        trajectories.add(traj3);
        trajectories.add(traj4);
        trajectories.add(traj5);
        trajectories.add(traj6);
        trajectories.add(traj7);
        trajectories.add(traj8);




    }

    public void  heartbeat() throws InterruptedException {
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }

    public void keepLiftAlive(int i) throws InterruptedException{

        if(true) {

            liftError = liftTargetPos - lift.getCurrentPosition();
            lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));
            liftB.setPower(lift.getPower());

            if(level != 0) {
                v4b1.setPosition(.81);
                v4b2.setPosition(.81);
                dep.setPosition(.46);
            }


            if (aman && !drive.isBusy()) {
                    dep.setPosition(.23);

                    sleep(450);

                    starts();

                    liftTargetPos = 0;
                    liftError = liftTargetPos - lift.getCurrentPosition();

                    lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));
                    liftB.setPower(lift.getPower());

                    aman = false;
                    reading = Distance.getDistance(DistanceUnit.MM);

                }

        }

    }


    public void safeGuard(){
            intake.setPower(intakePower);
            intakeB.setPower(intakePower);

            reading = Distance.getDistance(DistanceUnit.MM);

            telemetry.addLine("" + reading);
            telemetry.update();

            if (reading < 50) { //100
                intakePower = 0.5;
                intake.setPower(intakePower);
                intakeB.setPower(intakePower);
            }
    }




    public void starts(){
        v4b1.setPosition(.18);
        v4b2.setPosition(.18);
        dep.setPosition(.63);
        intake.setPower(0);
        intakeB.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();


        waitForStart();

        if (isStopRequested()) return;

        //Preload deposit
        drive.followTrajectorySequenceAsync(trajectories.get(0));

        //Runs the lift while going to the desired preload position
        while(aman){
            drive.update();
            keepLiftAlive(0);
        }

        starts();

        //Lets us know when to turn the intake on
        ElapsedTime whenToTurn = new ElapsedTime();

        for(int i = 1; i < 7; i+=2){
            drive.followTrajectorySequenceAsync(trajectories.get(i));

            liftTargetPos = 0;
            level = 0;

            whenToTurn.reset();

            //Goes to the warehouse, keeps the lift at the low position, and
            while(drive.isBusy()){
                drive.update();
                if(whenToTurn.milliseconds() > 900)
                    safeGuard();
                keepLiftAlive(i);
            }

            liftTargetPos = top;
            level = 3;
            aman = true;

            drive.followTrajectorySequenceAsync(trajectories.get(i + 1));

            ElapsedTime turnOffIntake = new ElapsedTime();
            while(drive.isBusy()){
                drive.update();
                keepLiftAlive(i + 1);
                intakePower = 0;

                if(turnOffIntake.milliseconds() > 2250){
                    intake.setPower(0);
                    intakeB.setPower(0);
                }
            }
            starts();
            intakePower = -1;
            }



        liftTargetPos = 0;
        level = 0;
        aman = false;

       while(opModeIsActive()){
           keepLiftAlive(0);
       }

    }





}

