package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.PositionalDeviceTracker;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.CubeDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
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


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.firstinspires.ftc.teamcode.LiftPID;
import org.firstinspires.ftc.teamcode.PID;
import org.firstinspires.ftc.teamcode.SkystoneDeterminationPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;



@Autonomous(name = "RedLeft")
public class RedLeft extends LinearOpMode //creates class
{ //test test
    BNO055IMU imu;

    private int level = 0;

    private DcMotorEx lift, liftB;
    private Servo v4b1, v4b2, dep;
    private CRServo duccL, duccR;

    private ElapsedTime extend = new ElapsedTime();

    final int liftGrav = (int) (9.8 * 3);
    private LiftPID liftPID = new LiftPID(-.03, 0, 0);
    private int liftError = 0;
    private int liftTargetPos = 0;

    private final int top = 950;
    private final int med = 476;


    private WebcamName weCam;
    private OpenCvCamera camera;
    private CubeDetectionPipeline pipeline;

    private SampleMecanumDrive drive;

    public void initialize() {

        drive = new SampleMecanumDrive(hardwareMap);


        //  intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        lift = (DcMotorEx) hardwareMap.dcMotor.get("LI");
        //intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        int ones = 0;
        int twos = 0;
        int threes = 0;

        while (!opModeIsActive()) {
            int bLevel = getLevel();
            if(bLevel == 1)
                ones++;
            else if(bLevel == 2)
                twos++;
            else
                threes++;
        }

        if(ones > 1)
            level = 1;
        else if(twos > 1)
            level = 2;
        else
            level = 3;

        telemetry.addData("DETECTED LEVEL: ",level);
        telemetry.update();

        liftTargetPos = top;


        //liftTargetPos = top; //We might need to change this
        liftError = liftTargetPos - lift.getCurrentPosition();


    }

    public int getLevel() {
        int ones = 0;
        int twos = 0;
        int threes = 0;
        int num = pipeline.getCubeNum();
        telemetry.addData("num of cubes",num);
        telemetry.update();
        for (int i = 0; i < num; i++) {
            try {
                if (pipeline.getHeight(i) > 150)
                    if (pipeline.getX(i) > 150)
                        return 2;
                    else if ((pipeline.getX(i) < 150) && (pipeline.getX(i) > 0))
                        return 1;
            }
            catch(Exception e){
                telemetry.addData("exception",0);
                telemetry.update();
            }
        }
        return 3;
    }

    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }



    public void liftAndDeposit() throws InterruptedException{

        liftError = liftTargetPos - lift.getCurrentPosition();

        boolean depositRun = true;
        ElapsedTime deposit = new ElapsedTime();
        ElapsedTime help = new ElapsedTime();

        while(depositRun){
            liftError = liftTargetPos - lift.getCurrentPosition();

            //Takes the lift up
            lift.setPower(Range.clip(liftPID.getCorrection(liftError), -1, 1));
            liftB.setPower(lift.getPower());

            telemetry.addData("Target Position", liftTargetPos);
            telemetry.addData("Current position", lift.getCurrentPosition());
            telemetry.update();

            if(Math.abs(liftError) < 50){


                if(extend.milliseconds() > 750 && extend.milliseconds() < 1500) {

                    //Moves the virtual bars forward
                    v4b1.setPosition(.79);
                    v4b2.setPosition(.79);
                }

                sleep(2000);

                if(extend.milliseconds() > 1500 && extend.milliseconds() < 2250 ) {
                    //Opens the deposit
                    dep.setPosition(.5);
                }
                if(extend.milliseconds() > 2250 && extend.milliseconds() < 3000 ) {

                    //Closes the deposit
                    dep.setPosition(.4);
                }
                if(extend.milliseconds() > 3750 && extend.milliseconds() < 4500 ) {

                    //Moves the virtual bars backward
                    v4b1.setPosition(.19);
                    v4b2.setPosition(.19);
                }
                if(extend.milliseconds() > 4500 && extend.milliseconds() < 5250 ) {

                    //Gravity pulls the lift down
                    lift.setPower(0);
                    liftB.setPower(lift.getPower());

                    depositRun = false;
                }

            }
        }


    }


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        redLeft();

    }

    public void redLeft() throws InterruptedException{
        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(3)
                .build();

        drive.followTrajectory(traj3);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(3,0))
                .strafeRight(19)
                .build();

        drive.followTrajectory(traj);

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(3,-19))
                .forward(15.5)
                .build();

        drive.followTrajectory(traj2);

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(18.5,-19))
                .back(11)
                .build();

        drive.followTrajectory(traj4);

        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(7.5,-19))
                .strafeLeft(44.5)
                .build();

        drive.followTrajectory(traj5);

        spinDuck();

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(7.5,27.5))
                .forward(21)
                .build();

        drive.followTrajectory(traj6);

        Trajectory traj7 = drive.trajectoryBuilder(new Pose2d(28.5,27))
                .strafeLeft(2)
                .build();

        drive.followTrajectory(traj7);

    }


    public void spinDuck() throws InterruptedException{
        ElapsedTime spinTime = new ElapsedTime();
        duccL.setPower(-0.2);
        duccR.setPower(-0.2);
        while (spinTime.milliseconds() <= 6000)
            heartbeat();
        duccL.setPower(0);
        duccR.setPower(0);

    }
}

