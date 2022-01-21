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

import java.util.ArrayList;


@Autonomous(name = "BlueLeft")
public class BlueLeft extends LinearOpMode //creates class
{ //test test
    BNO055IMU imu;

    private int level = 0;

    private DcMotorEx lift, liftB;
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

        ArrayList<Integer> levels = new ArrayList<Integer>();
        while (!opModeIsActive()) {

            int bLevel = getLevel();

            levels.add(bLevel);

            telemetry.addData("DETECTED LEVEL: ",bLevel);

            if(gamepad1.a)
                delay = true;


            telemetry.addData("Is delay turned on?", delay);
            telemetry.update();

        }

        level = 0;

        if(levels.contains(1)) {
            liftTargetPos = 0;
            level = 1;
        } else{
            for(int i = levels.size() - 30; i < levels.size(); i++) {
                if (levels.get(i) == 2) {
                    level = 2;
                    liftTargetPos = med;
                }
            }
        }

        if(level == 0) {
            level = 3;
            liftTargetPos =  top;
        }
        // liftTargetPos = top;

        //liftTargetPos = top; //We might need to change this
        liftError = liftTargetPos - lift.getCurrentPosition();


    }

    public int getLevel() {

        int num = pipeline.getCubeNum();

        for (int i = 0; i < num; i++) {
            try {
                telemetry.addData("Height", pipeline.getHeight(i));

                if ((pipeline.getHeight(i) > 100) && (pipeline.getY(i) > 400)) {
                    telemetry.addData("X Pos", pipeline.getX(i));
                    telemetry.addData("Y Pos", pipeline.getY(i));

                    if (pipeline.getX(i) > 150)
                        return 2;
                    else if ((pipeline.getX(i) < 150) && (pipeline.getX(i) > 0))
                        return 1;

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
        if(level == 1 || level == 2)
            targetV4B = .4;
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
        blueLeft();
        //liftAndDeposit();
    }

    public void blueLeft() throws InterruptedException{
        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(3)
                .build();

        drive.followTrajectory(traj3);

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(3,0))
                .strafeRight(21)
                .build();

        drive.followTrajectory(traj);




        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(3,-21))
                .forward(16)
                .build();

        drive.followTrajectory(traj2);

        liftAndDeposit();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(19,-21))
                .back(1.5)
                .build();

        //DO NOT MESS WITH ANYTHING HERE AFTER
        drive.followTrajectory(traj4);

        drive.turn(Math.toRadians(90));
        Trajectory traj5 = drive.trajectoryBuilder(new Pose2d(16,-21, Math.toRadians(90)))
                .forward(60)
                .build();

        drive.followTrajectory(traj5);


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

