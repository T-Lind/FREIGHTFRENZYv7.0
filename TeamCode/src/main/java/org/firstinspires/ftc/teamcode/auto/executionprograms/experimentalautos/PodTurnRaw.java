package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="PodTurnRaw")
public class PodTurnRaw extends LinearOpMode {
    private DcMotorEx left, right;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");


        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(!opModeIsActive()) {
            telemetry.addLine("Initialized.");
            telemetry.update();
        }

//        ElapsedTime t = new ElapsedTime();
//        double v = 1.15;
//        while( t.milliseconds()/1000 < 0.7){
//            KalmanFilter kLeft1 = new KalmanFilter(0);
//            PIDController pidLeft1 = new PIDController(0);
//
//            KalmanFilter kLeft2 = new KalmanFilter(0);
//            PIDController pidLeft2 = new PIDController(0);
//
//            double corL1 = pidLeft1.update((long)v, (long)kLeft1.filter(leftFront.getVelocity(RADIANS)));
//            double corL2 = pidLeft2.update((long)v, (long)kLeft2.filter(leftBack.getVelocity(RADIANS)));
//
//
//            leftFront.setVelocity(v+corL1, AngleUnit.RADIANS);
//            leftBack.setVelocity(v+corL2, AngleUnit.RADIANS);
//        }
//        leftFront.setVelocity(0);
//        leftBack.setVelocity(0);


    }
}