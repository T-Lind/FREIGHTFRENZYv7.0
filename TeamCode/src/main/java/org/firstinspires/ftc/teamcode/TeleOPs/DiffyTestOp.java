package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.support.Toggle;

@Disabled
@TeleOp(name="DiffyTestOp")
public class DiffyTestOp extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    final double VEL_COEFF = 4;

    Toggle toggleA;


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


        toggleA = new Toggle(false);


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";


    }

    @Override
    public void loop() {
        toggleA.updateLeadingEdge(gamepad1.a);

        drive();


        telemetry.update();

    }

    private double convert(double p){
        if(p > 0)
            return Math.log(p+1)+1;
        else if(p < 0)
            return -(Math.log(Math.abs(p)+1)+1);
        return 0;
    }

    public void drive() {
        // get the x and y values - reorient y to follow unit circle, of both sticks
        double left_stick_x = gamepad1.left_stick_x;
        double left_stick_y = -1*gamepad1.left_stick_y;
        double right_stick_x = gamepad1.right_stick_x;
        double right_stick_y = -1*gamepad1.right_stick_y;
        left_stick_x= convert(left_stick_x);
        left_stick_y= -convert(left_stick_y);
        right_stick_x= convert(right_stick_x);
        right_stick_y= -convert(right_stick_y);

        // VARIABLE TO REDUCE CODE
        double driveSimilarity = left_stick_x * -VEL_COEFF + left_stick_y * VEL_COEFF;

        // HIGH DEGREE OF USER CONTROL
        if(toggleA.getToggleState()){
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

            rightFront.setVelocity(1.025*rfv, AngleUnit.RADIANS);
            rightBack.setVelocity(rbv, AngleUnit.RADIANS);
        }


        // Add telemetry data
        telemetry.addData("LFv: ",leftFront.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("LBv: ",leftBack.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("RFv: ",rightFront.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("RFv: ",rightBack.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("a: ",toggleA.getToggleState());
    }
}
