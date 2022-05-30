package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.EasyToggle;


@TeleOp(name="DiffyTestOp")
public class DiffyTestOp extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    final double VEL_COEFF = 2;
    BNO055IMU imu;
    Orientation angles;

    EasyToggle toggleA;
    boolean a;

    @Override
    public void init() {
        // build the motor objects
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // toggle A is not in use at the moment
        toggleA = new EasyToggle("a", false, 1, false, false);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);

        a = false;
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // BAD TOGGLE - REPLACE LATER
        if(gamepad1.a && !a)
            a = true;
        else if(gamepad1.a && a)
            a = false;

        drive();

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        telemetry.update();

        toggleA.updateEnd();
    }

    public void drive() {
        // get the x and y values - reorient y to follow unit circle, of both sticks
        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = -1*gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;
        double right_stick_y = -1*gamepad1.right_stick_y;

        // VARIABLE TO REDUCE CODE
        double driveSimilarity = left_stick_x * -VEL_COEFF + left_stick_y * VEL_COEFF;

        // HIGH DEGREE OF USER CONTROL
        if(a){
            leftFront.setVelocity(driveSimilarity, AngleUnit.RADIANS);
            leftBack.setVelocity(left_stick_x*-VEL_COEFF-left_stick_y*2, AngleUnit.RADIANS);

            rightFront.setVelocity(right_stick_x*-VEL_COEFF-right_stick_y*VEL_COEFF, AngleUnit.RADIANS);
            rightBack.setVelocity(right_stick_x*-VEL_COEFF+right_stick_y*VEL_COEFF, AngleUnit.RADIANS);
        }

        // LOWER DEGREE OF USER CONTROL
        else{
            // set the velocity based on left stick
            double lfv = driveSimilarity;
            double lbv = left_stick_x * -VEL_COEFF - left_stick_y * VEL_COEFF;
            double rfv = left_stick_y * -VEL_COEFF - left_stick_x * VEL_COEFF;
            double rbv = -left_stick_y * -VEL_COEFF - left_stick_x * VEL_COEFF;

            // add turning motion
            lfv += right_stick_x*0.5*VEL_COEFF;
            lbv += -right_stick_x*0.5*VEL_COEFF;
            rfv += right_stick_x*0.5*VEL_COEFF;
            rbv += -right_stick_x*0.5*VEL_COEFF;

            // set velocities
            leftFront.setVelocity(lfv, AngleUnit.RADIANS);
            leftBack.setVelocity(lbv, AngleUnit.RADIANS);

            rightFront.setVelocity(rfv, AngleUnit.RADIANS);
            rightBack.setVelocity(rbv, AngleUnit.RADIANS);
        }



        // Add telemetry data
        telemetry.addData("LFv: ",leftFront.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("LBv: ",leftBack.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("RFv: ",rightFront.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("RFv: ",rightBack.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("a: ",a);
    }
}
