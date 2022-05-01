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
        KalmanFilter k3 = new KalmanFilter(0);
        PIDController pid3 = new PIDController(0);

        KalmanFilter k4 = new KalmanFilter(0);
        PIDController pid4 = new PIDController(0);


        double[] r = {1,0.7,1.2,1};
        double[] arcs = {0.4,-0.2,-0.4,0.1};

        ElapsedTime t2 = new ElapsedTime();
        t2.reset();

        SplinePath trajectory4 = new SplinePath(0.368,0.4,0.7,r,arcs);
        trajectory4.build();
        while(!trajectory4.getCompleted()){
            double leftV = convert(trajectory4.getLeftVelocity(t2.milliseconds()/1000));
            double rightV = convert(trajectory4.getRightVelocity(t2.milliseconds()/1000));
            telemetry.addData("left driven velocity: ", leftV);
            telemetry.addData("right driven velocity: ", rightV);
            telemetry.addData("path number ",trajectory4.getArc(t2.milliseconds()/1000));
            telemetry.update();
            double corL = pid3.update((long)leftV, (long)k3.filter(left.getVelocity(RADIANS)));
            double corR = pid4.update((long)rightV, (long)k4.filter(right.getVelocity(RADIANS)));

            left.setVelocity(corL+leftV, RADIANS);
            right.setVelocity(corR+rightV, RADIANS);
//            double[] vels = trajectory4.update(left.getVelocity(RADIANS), right.getVelocity(RADIANS), t2.milliseconds()/1000);
//            left.setVelocity(vels[0], RADIANS);
//            right.setVelocity(vels[1], RADIANS);
        }

    }
}
