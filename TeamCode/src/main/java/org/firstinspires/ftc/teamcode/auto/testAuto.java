package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

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

        /**
         * Code to move in a straight line.
         */
//        KalmanFilter k = new KalmanFilter(7,4);
//        PIDController pid = new PIDController(0.5,0.5,0.2);
//
//        KalmanFilter k2 = new KalmanFilter(7,4);
//        PIDController pid2 = new PIDController(0.5,0.5,0.2);
//
//        NeoPath trajectory3 = new Line(0.5,0.4);
//        trajectory3.build();
//
//
//        ElapsedTime t = new ElapsedTime();
//        t.reset();
//        while(!trajectory3.getCompleted()){
//            double leftV = convert(trajectory3.getLeftVelocity(t.milliseconds()/1000));
//            double rightV = convert(trajectory3.getRightVelocity(t.milliseconds()/1000));
//            telemetry.addData("left driven velocity: ", leftV);
//            telemetry.addData("right driven velocity: ", rightV);
//            telemetry.update();
//            leftV = k.filter(leftV);
//            rightV = k2.filter(rightV);
//            double corL = pid.update((long)leftV, (long)left.getVelocity(RADIANS));
//            double corR = pid.update((long)rightV, (long)right.getVelocity(RADIANS));
//
//            left.setVelocity(corL+leftV, RADIANS);
//            right.setVelocity(corR+rightV, RADIANS);
//        }

        /**
         * Splining code.
         */
        KalmanFilter k3 = new KalmanFilter(7,4);
        PIDController pid3 = new PIDController(0.3,0.3,0.2);

        KalmanFilter k4 = new KalmanFilter(7,4);
        PIDController pid4 = new PIDController(0.3,0.3,0.2);

        ElapsedTime t2 = new ElapsedTime();
        t2.reset();

        double[] r = new double[3];
        r[0] = 1.5;
        r[1] = 1;
        r[2] = 1.5;

        double[] arcs = new double[3];
        arcs[0] = 0.4;
        arcs[1] = 0.8;
        arcs[2] = 1.2;

        SplinePath trajectory4 = new SplinePath(0.368,0.4,0.75,r,arcs);
        trajectory4.build();
        while(!trajectory4.getCompleted()){
            double leftV = convert(trajectory4.getLeftVelocity(t2.milliseconds()/1000));
            double rightV = convert(trajectory4.getRightVelocity(t2.milliseconds()/1000));
            telemetry.addData("left driven velocity: ", leftV);
            telemetry.addData("right driven velocity: ", rightV);
            telemetry.update();
            leftV = k3.filter(leftV);
            rightV = k4.filter(rightV);
            double corL = pid3.update((long)leftV, (long)left.getVelocity(RADIANS));
            double corR = pid4.update((long)rightV, (long)right.getVelocity(RADIANS));

            left.setVelocity(corL+leftV, RADIANS);
            right.setVelocity(corR+rightV, RADIANS);
        }

    }
}
