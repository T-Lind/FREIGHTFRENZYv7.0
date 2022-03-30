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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;


@Autonomous(name = "BlueRight")
public class BlueRight extends LinearOpMode //creates class
{ //test test
    BNO055IMU imu;
    //test
    private RevColorSensorV3 color;
    private int level = 0;
    private boolean aman = true;
    private double targetV4B;
    private ArrayList<TrajectorySequence> trajectories;
    private DcMotorEx intake, lift, ducc;

    private Servo arm1, arm2, dep, fold;

    private double targetDeposit;
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

    public void initialize() throws InterruptedException {

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(90)));





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
        //initialize the color sensor
        color = hardwareMap.get(RevColorSensorV3.class, "color");



        arm1.setDirection(Servo.Direction.REVERSE);

        drive = new SampleMecanumDrive(hardwareMap);




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

        //initializeTrajectories();

        liftError = liftTargetPos - lift.getCurrentPosition();




    }

    public void initializeTrajectories() throws InterruptedException{
        trajectories = new ArrayList<TrajectorySequence>();

        double y = -40.5;

        if (level == 1)
            y = y - 2.8;

        //Initial deposit (do remember that I added .turn in the sequence, instead of separately doing that)
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                .setAccelConstraint((a,e,c,d)->45)
                .lineTo(new Vector2d(-24, y))
                .turn(Math.toRadians(-195))
                .build();

        //Go to duck and begin spinning
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(-24, y, Math.toRadians(-105)))
                .splineTo(new Vector2d(-63, -58.5), Math.toRadians(-90))
                .build();

        //Sweep trajectory
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(-63, -58.5, Math.toRadians(-90)))
                .strafeLeft(20)
                .turn(Math.toRadians(-17))
                .setAccelConstraint((a,e,c,d)->7)
                .lineTo(new Vector2d(-60,-62))
                .build();

        //Final duck deposit
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .setReversed(true)
                .setAccelConstraint((a,e,c,d)->20)
                .setVelConstraint((a,e,c,d)->40)
                .splineTo(new Vector2d(-33, -24), Math.toRadians(0))
                .build();

        //Code to park
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .lineTo(new Vector2d(-63, -39))
                .build();

        trajectories.add(traj1);
        trajectories.add(traj2);
        trajectories.add(traj3);
        trajectories.add(traj4);
        trajectories.add(traj5);
    }


    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }



    /*public void keepLiftAlive() throws InterruptedException{

        if(true) {

            liftError = liftTargetPos - lift.getCurrentPosition();
            lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));

            if(level == 1) {
                // v4b1.setPosition(.81);
                //v4b2.setPosition(.81);

                liftTargetPos=10;
                dep.setPosition(.46);
            }

            else if(level==2){
                //v4b1.setPosition(.7);
                //v4b2.setPosition(.7);

                liftTargetPos=med;
                dep.setPosition(.46);
            }

            else if(level==3) {
                //v4b1.setPosition(.81);
//                v4b2.setPosition(.81);

                liftTargetPos=top;
                dep.setPosition(.46);
            }


            if (aman && !drive.isBusy()) {
                dep.setPosition(.23);

                sleep(450);

                starts();

                liftTargetPos = 0;
                liftError = liftTargetPos - lift.getCurrentPosition();

                lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));


                aman = false;

            }

        }

    }*/


    public void starts(){
        fold.setPosition(.5);
        dep.setPosition(.57);
        arm1.setPosition(.5);
        arm2.setPosition(.5);


        intake.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        if(delay){
            sleep(5000);
        }
        starts();
        drive.setPoseEstimate(new Pose2d(-36, 63, Math.toRadians(90)));
        blueRight();
/*        drive.followTrajectorySequenceAsync(trajectories.get(0));

        while(aman){
            drive.update();
            keepLiftAlive();
        }

        for(int i = 1; i < trajectories.size(); i++){
            drive.followTrajectorySequenceAsync(trajectories.get(i));

            level = 0;
            aman = false;

            if(i == 2){
                intake.setPower(-1);
            }

            if(i == 3){
                dep.setPosition(.46);
                level = 3;
                liftTargetPos = top;
                aman = true;
            }

            while(drive.isBusy()){
                drive.update();
                keepLiftAlive();
            }

            if(i == 1)
                spinDuck();

            intake.setPower(0);
            intake.setPower(0);
        }
*/
    }

    public void blueRight() throws InterruptedException{


        if (isStopRequested()) return;
        TrajectorySequence toHub = drive.trajectorySequenceBuilder(new Pose2d(-36, 63, Math.toRadians(-90)))
                .splineTo(new Vector2d(-28,24), Math.toRadians(0))

                .build();
        drive.followTrajectorySequence(toHub);
        //deposit(getduccLevel());
        TrajectorySequence toDuck = drive.trajectorySequenceBuilder(toHub.end())
                .setReversed(true)
                .splineTo(new Vector2d(-60, 60),Math.toRadians(-215))

                .build();
        drive.followTrajectorySequence(toDuck);
        spinDuck();
        TrajectorySequence suckDuckAndToHub = drive.trajectorySequenceBuilder(toDuck.end())
                .setReversed(false)
                .turn(Math.toRadians(-55))
                .strafeLeft(20)
                .turn(Math.toRadians(17))
                // .setAccelConstraint((a,e,c,d)->7)
                .lineTo(new Vector2d(-60,62))
                .turn(Math.toRadians(17))
                .splineTo(new Vector2d(-33, 24), Math.toRadians(0))

                .build();
        drive.followTrajectorySequence(suckDuckAndToHub);
        //deposit(3);
        TrajectorySequence park = drive.trajectorySequenceBuilder(suckDuckAndToHub.end())
                .setReversed(true)
                .splineTo(new Vector2d(-65,37),Math.toRadians(180))

                .build();
        drive.followTrajectorySequence(park);
/*
                //.turn(Math.toRadians(-45))
                .setReversed(true)
                .splineTo(new Vector2d(-60, -59),Math.toRadians(215))//go to ducc
                .setReversed(false)
                //.turn(Math.toRadians(-10))//get better angle to ducc

                .build();
        drive.followTrajectorySequence(traj1);
        spinDuck();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(55))
                //.turn(Math.toRadians(-10))//get better angle to ducc
                // .turn(Math.toRadians(0))
                // .lineTo(new Vector2d(-40, -62))//strafe ducc
                .strafeRight(20)
                .turn(Math.toRadians(-17))
                // .setAccelConstraint((a,e,c,d)->7)
                .lineTo(new Vector2d(-60,-62))
                .turn(Math.toRadians(17))
                .splineTo(new Vector2d(-33, -24), Math.toRadians(0))
                .setReversed(true)
                .splineTo(new Vector2d(-65,-37),Math.toRadians(180))
                .build();
        drive.followTrajectorySequence(traj2);*/
    }

    public void spinDuck() throws InterruptedException {
        ElapsedTime spinTime = new ElapsedTime();
        ducc.setPower(-.3);
        while(spinTime.milliseconds()<=3000){
            heartbeat();
        }
        ducc.setPower(0);
    }
}

