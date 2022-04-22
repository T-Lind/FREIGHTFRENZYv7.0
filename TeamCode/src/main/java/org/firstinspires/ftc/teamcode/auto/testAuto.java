package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous(name="testAuto")
public class testAuto extends LinearOpMode {
    private DcMotorEx left, right;

    protected double convert(double v){
        v/=0.048; // convert to angular velocity by radius
        v/=(2*3.14159);
        v*=537.7;
        return v;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        left = (DcMotorEx) hardwareMap.dcMotor.get("L");
        right = (DcMotorEx) hardwareMap.dcMotor.get("R");

        while(!opModeIsActive()) {
            telemetry.addData("working", " pretty well");
            telemetry.update();
        }

        KalmanFilter k = new KalmanFilter(10,4);
        PIDController pid = new PIDController(0.3,0.15,0.5);

        KalmanFilter k2 = new KalmanFilter(10,4);
        PIDController pid2 = new PIDController(0.3,0.15,0.5);

        PIDKController pidControllerL = new PIDKController(k,pid);
        PIDKController pidControllerR = new PIDKController(k2,pid2);

        NeoPath trajectory3 = new Line(1,0.8);


        ElapsedTime t = new ElapsedTime();
        t.reset();
        while(!trajectory3.getCompleted()){
            double leftV = convert(trajectory3.getLeftVelocity(t.milliseconds()/1000));
            double rightV = convert(trajectory3.getRightVelocity(t.milliseconds()/1000));
            telemetry.addData("left driven velocity: ", leftV);
            telemetry.addData("right driven velocity: ", rightV);
            double leftCorrected = pidControllerL.update((long)leftV,(long)left.getVelocity());
            double rightCorrected = pidControllerR.update((long)rightV,(long)right.getVelocity());
            telemetry.addData("left corrected velocity: ", leftCorrected);
            telemetry.addData("right corrected velocity: ", rightCorrected);
            telemetry.update();

            left.setVelocity(leftV);
            right.setVelocity(rightV);
        }



    }
}
