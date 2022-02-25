package org.firstinspires.ftc.teamcode.TeleOPs;

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

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.EasyToggle;

// this is a test teleop class for testing. Do not use in competition. - Seb on may 7th, 2021.
@TeleOp(name="DebugOp")
public class DebugOp extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, intakeB, lift, liftB;
    private Servo v4b1, v4b2, dep, duccRot, duccTilt;
    private CRServo duccL, duccR, duccEx;
    private boolean direction, togglePrecision;
    private double factor;
    boolean reverse;
    private Rev2mDistanceSensor Distance;
    EasyToggle toggleA = new EasyToggle("a", false, 1, false, false);
    double rot = .5;
    double tilt = .5;
    double extend = 0;


    @Override
    public void init() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        intake = (DcMotorEx) hardwareMap.dcMotor.get("IN");
        lift = (DcMotorEx) hardwareMap.dcMotor.get("LI");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lift.setDirection(DcMotor.Direction.REVERSE);

        liftB = (DcMotorEx) hardwareMap.dcMotor.get("LIB");
        liftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        liftB.setDirection(DcMotor.Direction.REVERSE);

        intakeB = (DcMotorEx) hardwareMap.dcMotor.get("INB");
        intakeB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        v4b1 = hardwareMap.servo.get("v4b1");
        v4b2 = hardwareMap.servo.get("v4b2");
        dep = hardwareMap.servo.get("dep");
        duccL = hardwareMap.crservo.get("DL");
        duccR = hardwareMap.crservo.get("DR");

        duccRot = hardwareMap.servo.get("DRoT");
        duccTilt = hardwareMap.servo.get("DT");
        duccEx = hardwareMap.crservo.get("DE");

        duccL.setDirection(DcMotorSimple.Direction.REVERSE);

        v4b1.setDirection(Servo.Direction.REVERSE);

        Distance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "detect");






    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            leftBack.setPower(.1);
        } else {
            leftBack.setPower(0);
        }
        if(gamepad1.b) {
            rightBack.setPower(.1);
        }else {
            rightBack.setPower(0);
        }
        if(gamepad1.x) {
            leftFront.setPower(.1);
        }else {
            leftFront.setPower(0);
        }
        if(gamepad1.y) {
            rightFront.setPower(.1);
        }else {
            rightFront.setPower(0);
        }

        if (gamepad1.left_trigger > .5) {
            intake.setPower(-.5);
            intakeB.setPower(-.5);
        } else if (gamepad1.left_bumper) {
            intake.setPower(.5);
            intakeB.setPower(.5);
        } else {
            intake.setPower(0);
            intakeB.setPower(0);
        }

        extend = gamepad1.left_stick_y;
        if(extend < .2 && extend > .2)
            extend = 0;
        if(gamepad1.left_stick_x > .3)
            rot += .005;
        else if(gamepad1.left_stick_x < -.3)
            rot -= .005;
        if(gamepad1.right_stick_y > .3)
            tilt -= .005;
        else if(gamepad1.right_stick_y < -.3)
            tilt += .005;

        duccEx.setPower(Range.clip(extend, -1, 1));
        duccRot.setPosition(Range.clip(rot, -1, 1));
        duccTilt.setPosition(Range.clip(tilt, -1, 1));


        telemetry.addData("RF", rightFront.getCurrentPosition());
        telemetry.addData("RB", rightBack.getCurrentPosition());
        telemetry.addData("LF", leftFront.getCurrentPosition());
        telemetry.addData("LB", leftBack.getCurrentPosition());
        telemetry.addData("intake current", intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("intakeB current", intakeB.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("rot", duccRot.getPosition());
        telemetry.addData("tilt", duccTilt.getPosition());
        telemetry.speak("sussy sebby");
        telemetry.update();

    }
}

