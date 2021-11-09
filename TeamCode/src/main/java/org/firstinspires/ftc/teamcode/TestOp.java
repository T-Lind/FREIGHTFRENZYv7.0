package org.firstinspires.ftc.teamcode;

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
//not paying attention in CS2 pog

// this is a test teleop class for testing. Do not use in competition. - Seb on may 7th, 2021.
@TeleOp(name="TestOp")
public class TestOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, lift, liftB;
    private Servo v4b1, v4b2, dep;
    private CRServo duccL, duccR;
    private boolean direction, togglePrecision;
    private double factor;
    //test
    boolean reverse;
    private Rev2mDistanceSensor Distance;
    EasyToggle toggleA = new EasyToggle("a", false, 1, false, false);


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
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

        duccL.setDirection(DcMotorSimple.Direction.REVERSE);

        v4b1.setDirection(Servo.Direction.REVERSE);

        Distance = (Rev2mDistanceSensor) hardwareMap.get(DistanceSensor.class, "spit");



    }

    @Override
    public void start() {
        v4b1.setPosition(.79);
        v4b2.setPosition(.79);
        dep.setPosition(.43);
    }

    @Override
    public void loop() {
        toggleA.updateStart(gamepad1.a);
        //toggles precision mode if the right stick button is pressed
        if (gamepad1.left_stick_button)
            togglePrecision = true;
        else if (gamepad1.right_stick_button)
            togglePrecision = false;

        //sets the factor multiplied to the power of the motors
        factor = togglePrecision ? .3 : 1; //the power is 1/5th of its normal value while in precision mode

        // Do not mess with this, if it works, it works
        double x = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double stickAngle = Math.atan2(direction ? -gamepad1.left_stick_y : gamepad1.left_stick_y, direction ? gamepad1.left_stick_x : -gamepad1.left_stick_x); // desired robot angle from the angle of stick
        double powerAngle = stickAngle - (Math.PI / 4); // conversion for correct power values
        double rightX = -gamepad1.right_stick_x; // right stick x axis controls turning
        final double leftFrontPower = Range.clip(x * Math.cos(powerAngle) - rightX, -1.0, 1.0);
        final double leftRearPower = Range.clip(x * Math.sin(powerAngle) - rightX, -1.0, 1.0);
        final double rightFrontPower = Range.clip(x * Math.sin(powerAngle) + rightX, -1.0, 1.0);
        final double rightRearPower = Range.clip(x * Math.cos(powerAngle) + rightX, -1.0, 1.0);

        //leftFront.setDirection(DcMotor.Direction.FORWARD);
        //leftBack.setDirection(DcMotor.Direction.FORWARD);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        //rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setPower(leftFrontPower * factor);
        leftBack.setPower(leftRearPower * factor);
        rightFront.setPower(rightFrontPower * factor);
        rightBack.setPower(rightRearPower * factor);

        speak();
        lift();
        succ();
        duccSpin();
        deposit();


        telemetry.addData("lift", lift.getCurrentPosition());

        telemetry.update();
        toggleA.updateEnd();

        //deadwheel time
        // deadwheels were a lie :(
    }

    public void succ() {
        if (gamepad1.left_trigger > .5) {
            intake.setPower(-1);
        } else if (gamepad1.left_bumper) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }

    }

    public void speak() {
        if (gamepad1.dpad_left) {
            telemetry.speak("FOR JON IN JON ON JON");
        }
    }

    public void lift() {
        if (gamepad1.dpad_up) {
            lift.setPower(-.6);
            liftB.setPower(-.6);
        } else if (gamepad1.dpad_down) {
            lift.setPower(.01);
            liftB.setPower(.01);
        } else {
            lift.setPower(0);
            liftB
                    .setPower(0);
        }
    }

    public void duccSpin() {
        if (gamepad1.a) {
            duccL.setPower(1);
            duccR.setPower(1);
        } else {
            duccL.setPower(0);
            duccR.setPower(0);
        }
    }

    public void deposit() {
        if (gamepad1.x) {
            v4b1.setPosition(.19);
            v4b2.setPosition(.19);
            //intake position
        } else if (gamepad1.y) {
            v4b1.setPosition(.79);
            v4b2.setPosition(.79);
            //deposit position
        } else if (gamepad1.b) {
            v4b1.setPosition(.5);
            v4b2.setPosition(.5);
            //vertical position for asserting dominance
        }

        if (gamepad1.right_trigger > .5) {
            dep.setPosition(.5);
        } else {
            dep.setPosition(.43);
        }
    }

}