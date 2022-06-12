package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="ExperimentalSpeedTest")
public class ExperimentalSpeedTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DcMotorEx leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        DcMotorEx leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(!opModeIsActive());

        waitForStart();

        leftFront.setVelocity(1.36/2, AngleUnit.RADIANS);
        leftBack.setVelocity(-1.36/2, AngleUnit.RADIANS);

        ElapsedTime timer = new ElapsedTime();
        double currentTime = timer.milliseconds()/1000;
        while(currentTime < 2){
            currentTime = timer.milliseconds()/1000;
        }
        leftFront.setVelocity(0);
        leftBack.setVelocity(0);
    }
}
