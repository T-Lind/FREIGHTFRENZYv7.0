package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.EasyToggle;
import org.firstinspires.ftc.teamcode.PIDS.LiftPID;

// this is a test teleop class for testing. Do not use in competition. - Seb on may 7th, 2021.
@TeleOp(name="DebugOp")
public class DebugOp extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, lift, ducc;
    private Servo arm1, arm2, dep, fold;
    private boolean direction, togglePrecision;
    double p = .02;
    double d = .01;
    private double targetV4B = 0.81;
    private LiftPID liftPID = new LiftPID(p, 0, d);
    Orientation angles;
    double foldI = .83;
    private double factor;
    //test
    boolean reverse;
    BNO055IMU imu;
    int liftError = 0;
    int liftTargetPos = 1000;

    EasyToggle toggleA = new EasyToggle("a", false, 1, false, false);
    EasyToggle toggleB = new EasyToggle("b", false, 1, false, false);
    EasyToggle toggleUp = new EasyToggle("up", false, 1, false, false);
    EasyToggle toggleDown = new EasyToggle("down", false, 1, false, false);
    EasyToggle levelUp = new EasyToggle("lu", false, 1, false, false);
    EasyToggle levelDown = new EasyToggle("ld", false, 1, false, false);


    @Override
    public void init(){
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

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



        arm1.setDirection(Servo.Direction.REVERSE);



        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        arm1.setPosition(.5);
        arm2.setPosition(.5);

    }

    @Override
    public void loop() {
        toggleUp.updateStart(gamepad1.right_bumper/*gamepad2.dpad_up*/);
        toggleDown.updateStart(gamepad1.left_bumper/*gamepad2.dpad_down*/);
        levelUp.updateStart(gamepad1.dpad_up/*gamepad2.dpad_up*/);
        levelDown.updateStart(gamepad1.dpad_down/*gamepad2.dpad_down*/);
        toggleA.updateStart(gamepad1.a);
        toggleB.updateStart(gamepad1.b);
        if(levelUp.nowTrue()){
            p += .005;
        } else if (levelDown.nowTrue()){
            p -= .005;
        }
        if(toggleB.nowTrue()){
            d += .005;
        } else if(toggleA.nowTrue()){
            d -= .005;
        }
        liftPID.setPD(p,d);
        macroLift();

        telemetry.addData("p", p);
        telemetry.addData("d", d);
        telemetry.addData("lift position", lift.getCurrentPosition());
        telemetry.update();


        toggleUp.updateEnd();
        toggleDown.updateEnd();
        levelUp.updateEnd();
        levelDown.updateEnd();
        toggleA.updateEnd();
        toggleB.updateEnd();
    }

    public void macroLift() {
        liftError = liftTargetPos - lift.getCurrentPosition();
        lift.setPower(Range.clip(liftPID.getCorrection(liftError), -.02, 1));
        if (toggleUp.nowTrue()) {
            liftTargetPos = 1000;
            arm1.setPosition(.83);
            arm2.setPosition(.83);

        }
        if (toggleDown.nowTrue()) {
            liftTargetPos = 0;
            arm1.setPosition(.5);
            arm2.setPosition(.5);
        }

    }
}

