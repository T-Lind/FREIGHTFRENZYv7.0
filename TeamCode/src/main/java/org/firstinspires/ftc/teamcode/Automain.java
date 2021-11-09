package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous
public class Automain extends LinearOpMode //creates class
{ //test test
    BNO055IMU imu;
    private DcMotorEx leftFront, rightFront, leftBack, rightBack;
    //private HardwareMap hardwareMap;
    private final double WHEEL_RADIUS = 3.77953;
    private final double GEAR_RATIO = (double) 1;
    private final double TICKS_PER_REVOLUTION = 537.6;
    Orientation angles;
    private DcMotorEx[] motors;
    private double initialAngle;
    private ElapsedTime runtime;

    private final double TPI = TICKS_PER_REVOLUTION / (2 * Math.PI * GEAR_RATIO * WHEEL_RADIUS);
    private PID forwardPID = new PID(.022, 0, 0.0033);
    private PID strafePID = new PID(.024, 0, 0.0033);

    private WebcamName weCam;
    private OpenCvCamera camera;
    private UltiamteGoalDeterminationPipeline pipeline;

    public void initialize(){

        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");
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
        motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        initialAngle = angles.firstAngle;

        runtime = new ElapsedTime();

       weCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(weCam, cameraMonitorViewId);
        pipeline = new UltiamteGoalDeterminationPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            // @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
        });



    }

    public void moveBot(int[] dir, int distance, double power, boolean withIntake) throws InterruptedException {
        //moveBot(1, 1, 2, 2, -24, .60, true); //Forwward
        //turnBot(2, 2, 1, 1, -24, .60); //Backward
        //turnBot(2, 1, 2, 1, 30, .60); //Strafe right
        //turnBot(1, 2, 1, 2, 0, .6) /Strafe left
        int leftT = dir[0];
        int rightT = dir[1];
        int leftB = dir[2];
        int rightB = dir[3]; // MAKE CASE FOR 0
        if (leftT == 1) {
            leftFront.setDirection(DcMotor.Direction.FORWARD);
        } else if (leftT == 2) {
            leftFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (leftB == 1) {
            leftBack.setDirection(DcMotor.Direction.FORWARD);
        } else if (leftB == 2) {
            leftBack.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightT == 2) {
            rightFront.setDirection(DcMotor.Direction.FORWARD);
        } else if (rightT == 1) {
            rightFront.setDirection(DcMotor.Direction.REVERSE);
        }

        if (rightB == 2) {
            rightBack.setDirection(DcMotor.Direction.FORWARD);
        } else if (rightT == 1) {
            rightBack.setDirection(DcMotor.Direction.REVERSE);
        }

        /*if(withIntake) {
            while (shooterTime.milliseconds() <= 5000) {
                intake.setPower(.5);
                transfer.setPower(1);
                heartbeat();
            }
        }*/

        //Moves the robot
        int travel = (int) (distance * TPI);
        for (DcMotorEx motor : motors) {
            motor.setTargetPosition(travel);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setPower(power);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        //This is what checks if the motors are supposed to be still running.
        while (leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy()) {
            heartbeat();
        }
        //intake.setPower(0);
        // transfer.setPower(0);
    }

    public void initializeMotors(){
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            if (motor == leftFront || motor == leftBack)
                motor.setDirection(DcMotor.Direction.REVERSE);
            else
                motor.setDirection(DcMotor.Direction.FORWARD);
        }
        waitForStart();
    }
    public void moveByWheelEncoders(double targetHeading, double inches, double power, String movementType) throws InterruptedException {
        resetMotors();

        double currentPosition = leftBack.getCurrentPosition();

        int ticks = (int)(inches * TPI);

        //heartbeat isn't functional as of now

        while (motorsBusy(ticks, currentPosition)) {
            heartbeat();
            correction(-power, targetHeading, movementType, false, 1);
        }

        halt();
    }
    public void strafeByWheelEncoders(double targetHeading, double inches, double power, String movementType) throws InterruptedException {
        resetMotors();

        double currentPosition = leftBack.getCurrentPosition();

        int ticks = (int)(inches * TPI * 1.2);

        //Heartbeat isn't functional as of now- should be fixed now 10/18

        while (motorsBusy(ticks, currentPosition)) {
            heartbeat();
            correction(-power, targetHeading, movementType, false, 1);
        }

        halt();
    }

    public void resetEncoders() throws InterruptedException{
        for (DcMotorEx motor : motors) {
            if (motor == leftFront || motor == leftBack)
                motor.setDirection(DcMotor.Direction.REVERSE);
            else
                motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }
    public void resetMotors() throws InterruptedException{
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void moveForward() throws InterruptedException{
        int[] arr = new int[]{1, 1, 1, 1};
        moveBot(arr, 10, 0.5, false);
    }
    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }


    public boolean motorsBusy(int ticks, double startingPosition) {
        return Math.abs(leftBack.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightBack.getCurrentPosition() - startingPosition) < ticks && Math.abs(leftFront.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightFront.getCurrentPosition() - startingPosition) < ticks;
    }
    public double getError(double current, double target) {
        double error;

        double error1 = /*current - target*/ target - current;
        double error2;
        if (current < 0)
            error2 = /*(current + 360) - (target);*/ target - (current + 360);
        else
            error2 = /*(current - 360) - (target);*/ target - (current - 360);
        if (Math.abs(error1) <= Math.abs(error2))
            error = error1;
        else
            error = error2;

        return error;
    }
    public double currentAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
    public void correction(double power, double targetHeading, String movementType, boolean inverted, double max) throws InterruptedException {
        //sets target and current angles
        double target = targetHeading;
        double current = currentAngle();

        //if the spline motion is backwards, the target must be flipped 180 degrees in order to match with spline.getAngle()
        if (inverted && movementType.contains("spline")) {
            target = (targetHeading > 0) ? (targetHeading - 180) : (targetHeading + 180);
        }

        //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
        double error = getError(current, target);

        //PD correction for both regular and spline motion
        if (movementType.contains("straight") || movementType.contains("spline")) {
            double correction = forwardPID.getCorrection(error, runtime);

            double leftPower = Range.clip(power - correction, -max, max);
            double rightPower = Range.clip(power + correction, -max, max);
            double nearing = 0;

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);
//            telemetry.addData("left expected power", leftPower);
//            telemetry.addData("right expected power", rightPower);
//            telemetry.addData("actual left power", leftFront.getPower());
//            telemetry.addData("actual right power", rightFront.getPower());
        }

        //pd correction for strafe motion. Right and left are opposites

        else if (movementType.contains("strafe")) {
            double correction = strafePID.getCorrection(error, runtime);

            if (movementType.contains("left")) {
                leftFront.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(power + correction, -1.0, 1.0));
                leftBack.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightBack.setPower(Range.clip(-power + correction, -1.0, 1.0));
            } else if (movementType.contains("right")) {
                leftFront.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(-power + correction, -1.0, 1.0));
                leftBack.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightBack.setPower(Range.clip(power + correction, -1.0, 1.0));
            }
        }
    }
    public void halt() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeMotors();
        //moveByWheelEncoders(0, 100, 0.5, "straight");

        moveBot(new int[]{1, 1, 1, 1}, 50, 0.5, false);
    }

}

