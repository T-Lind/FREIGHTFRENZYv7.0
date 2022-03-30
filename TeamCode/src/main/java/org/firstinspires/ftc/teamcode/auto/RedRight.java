package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
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


        telemetry.addData("DETECTED LEVEL: ",level);
        telemetry.update();

        camera.setPipeline(pipeline2);

        if(level == 0)
            level = 3;

        if(level == 1)
            liftTargetPos = 0;
         else if(level == 2)
            liftTargetPos = med;
        else
            liftTargetPos = top;

//        initializeTrajectories();

    }

    public void initializeTrajectories() throws InterruptedException{
        trajectories = new ArrayList<TrajectorySequence>();
        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));
        //Preload (0)
        double y = -39;
        if(level == 1){
            y -= 1.5;
        }
        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(11, -63, Math.toRadians(90)))
                .lineTo(new Vector2d(0, y))
                .turn(Math.toRadians(-140))
                .build();

        //Going back to the warehouse, first cycle (1)
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(new Pose2d(0, y, Math.toRadians(-50)))
                /*.setAccelConstraint((a,e,c,d)->30)
                .setVelConstraint((a,e,c,d)->40)*/
                .splineTo(new Vector2d(9,-57), Math.toRadians(0))

                .forward(47)
                .build();


        //Going to deposit freight, first cycle (2)
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(new Pose2d(55, -57, Math.toRadians(0)))
                .setReversed(true)
                .back(47)
                .setAccelConstraint((a,e,c,d)->30)
                .setVelConstraint((a,e,c,d)->40)
                .splineTo(new Vector2d(7, -37), Math.toRadians(110)) //10
                .build();


        //Going back to warehouse, second cycle (3)
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(new Pose2d(7, -37, Math.toRadians(-70)))//new Pose2d(-12, -42, Math.toRadians(90)))

                .splineTo(new Vector2d(8, -57), Math.toRadians(0)) //-57

                .forward(58.5)
                .build();

        //Going to deposit freight, second cycle (4)
        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(new Pose2d(66.5, -57, Math.toRadians(0)))
                .setReversed(true)
                .back(59)
               /* .setAccelConstraint((a,e,c,d)->30)
                .setVelConstraint((a,e,c,d)->40)*/
                .splineTo(new Vector2d(16, -36.5), Math.toRadians(110))
                .build();


        //Going to warehouse, third cycle (5)
        TrajectorySequence traj6 = drive.trajectorySequenceBuilder(new Pose2d(16, -38, Math.toRadians(-70)))
                .splineTo(new Vector2d(9, -60), Math.toRadians(0)) //-59
                .forward(64)
                .build();

        //Going to deposit freight, third cycle (6)
        TrajectorySequence traj7 = drive.trajectorySequenceBuilder(new Pose2d(73, -57, Math.toRadians(0))) //-59
                .back(54)
                /*.setAccelConstraint((a,e,c,d)->30)
                .setVelConstraint((a,e,c,d)->40)*/
                .splineTo(new Vector2d(15, -40.5), Math.toRadians(110))
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

/*    public void keepLiftAlive(int i) throws InterruptedException{

        if(true) {

            liftError = liftTargetPos - lift.getCurrentPosition();
            lift.setPower(Range.clip(liftPID.getCorrection(liftError), 0, 1));

            if(level != 0) {

                v4b1.setPosition(0.81);
                v4b2.setPosition(0.81);
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
*/


/*
    public void safeGuard(){
            intake.setPower(intakePower);
            intakeB.setPower(intakePower);

            reading = Distance.getDistance(DistanceUnit.MM);

            telemetry.addLine("" + reading);
            telemetry.update();

            boolean timeKeeper = (whenToTurn.milliseconds() - timer) > 1300;
            if (reading < 100 || timeKeeper) { //100
                intakePower = 0.7;
                intake.setPower(intakePower);
                intakeB.setPower(intakePower);
            }
    }

*/


    public void starts(){
        fold.setPosition(.5);
        dep.setPosition(.57);
        arm1.setPosition(.5);
        arm2.setPosition(.5);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

starts();
        redRight();
        if (isStopRequested()) return;
/*
        //Preload deposit
        drive.followTrajectorySequenceAsync(trajectories.get(0));

        //Runs the lift while going to the desired preload position
        while(aman){
            drive.update();
            keepLiftAlive(0);
        }

        starts();

        //Lets us know when to turn the intake on
        whenToTurn = new ElapsedTime();

        for(int i = 1; i < 7; i+=2){
            drive.followTrajectorySequenceAsync(trajectories.get(i));

            liftTargetPos = 0;
            level = 0;

            whenToTurn.reset();

            //Goes to the warehouse, keeps the lift at the low position, and
            while(drive.isBusy()){
                drive.update();
                if(whenToTurn.milliseconds() > timer)
                    safeGuard();
                keepLiftAlive(i);
            }

            dep.setPosition(.46);
            sleep(150);

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
            intakePower = -0.85;
            timer = 2600;
            }



        liftTargetPos = 0;
        level = 0;
        aman = false;

        drive.followTrajectorySequenceAsync(trajectories.get(7));
       while(opModeIsActive()){
           drive.update();
           keepLiftAlive(0);
       }

    }

*/


}
    public void redRight() throws InterruptedException{

        drive.setPoseEstimate(new Pose2d(11, -63, Math.toRadians(90)));
        if (isStopRequested()) return;
        TrajectorySequence redWarehouse = drive.trajectorySequenceBuilder(new Pose2d(11,-63, Math.toRadians(90)))
                .splineTo(new Vector2d(2,-36), Math.toRadians(125))
                .setReversed(true)
                .splineTo(new Vector2d(14,-64), Math.toRadians(0))
                .strafeLeft(4.3)
                .setReversed(false)
                .back(35)
                .forward(30)
                .splineTo(new Vector2d(-1,-45), Math.toRadians(115))

                .build();

        drive.followTrajectorySequence(redWarehouse);
    }

}


