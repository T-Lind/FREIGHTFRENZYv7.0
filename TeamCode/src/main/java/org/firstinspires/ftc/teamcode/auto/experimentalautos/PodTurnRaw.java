package org.firstinspires.ftc.teamcode.auto.experimentalautos;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.support.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.PIDController;

@Autonomous(name="PodTurnRaw")
public class PodTurnRaw extends LinearOpMode {
    private DcMotorEx left, right;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    private double convertDistance(double meters){
        return 3.09*meters+0.224;
    }

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
//
//        ElapsedTime t = new ElapsedTime();
//        t.reset();
//        double v = 1.15;
//        while( t.milliseconds()/1000 < 0.8){
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
//        sleep(1000);
//
        // 0.92*2 velocity and 1/2 time results in 90 degree turn if both mpodules are being used 0.92 rad/s = 90 degrees/s
//        t.reset();
//        double v1 = 0.92*2;
//        while( t.milliseconds()/1000 < 1){
//            KalmanFilter kLeft1 = new KalmanFilter(0);
//            PIDController pidLeft1 = new PIDController(0);
//
//            KalmanFilter kLeft2 = new KalmanFilter(0);
//            PIDController pidLeft2 = new PIDController(0);
//
//            double corL1 = pidLeft1.update((long)v1, (long)kLeft1.filter(leftFront.getVelocity(RADIANS)));
//            double corL2 = pidLeft2.update((long)v1, (long)kLeft2.filter(leftBack.getVelocity(RADIANS)));
//
//
//            leftFront.setVelocity(v1+corL1, AngleUnit.RADIANS);
//        }

        //1 rad on each motor = 0.23m traveled
        //4.35 rad traveled = 1.35m
        //3.22 rad traveled = 1m
        //3.22/2 rad traveled = 0.43m
        //3*3.22/4 rad traveled = 0.72m
        //3.22/5 rad traveled = 0.11m

        ElapsedTime t = new ElapsedTime();
        t.reset();



        double v = 1;
        while( t.milliseconds()/1000 < convertDistance(0.60)){
            KalmanFilter kLeft1 = new KalmanFilter(0);
            PIDController pidLeft1 = new PIDController(0);

            KalmanFilter kLeft2 = new KalmanFilter(0);
            PIDController pidLeft2 = new PIDController(0);

            double corL1 = pidLeft1.update((long)v, (long)kLeft1.filter(leftFront.getVelocity(RADIANS)));
            double corL2 = pidLeft2.update((long)v, (long)kLeft2.filter(leftBack.getVelocity(RADIANS)));


            leftFront.setVelocity(-(v+corL1), AngleUnit.RADIANS);
            leftBack.setVelocity((v+corL2), AngleUnit.RADIANS);
            rightFront.setVelocity(-(v+corL2), AngleUnit.RADIANS);
            rightBack.setVelocity((v+corL2), AngleUnit.RADIANS);
        }
        leftFront.setVelocity(0);
        leftBack.setVelocity(0);



    }
}
