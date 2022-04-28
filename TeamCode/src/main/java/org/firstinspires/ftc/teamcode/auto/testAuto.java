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

        KalmanFilter k = new KalmanFilter(7,4);
        PIDController pid = new PIDController(0.5,0.5,0.2);

        KalmanFilter k2 = new KalmanFilter(7,4);
        PIDController pid2 = new PIDController(0.5,0.5,0.2);

        NeoPath trajectory3 = new Line(0.5,0.4);
        trajectory3.build();


        ElapsedTime t = new ElapsedTime();
        t.reset();
        while(!trajectory3.getCompleted()){
            double leftV = convert(trajectory3.getLeftVelocity(t.milliseconds()/1000));
            double rightV = convert(trajectory3.getRightVelocity(t.milliseconds()/1000));
            telemetry.addData("left driven velocity: ", leftV);
            telemetry.addData("right driven velocity: ", rightV);
            telemetry.update();
            leftV = k.filter(leftV);
            rightV = k2.filter(rightV);
            double corL = pid.update((long)leftV, (long)left.getVelocity(RADIANS));
            double corR = pid.update((long)rightV, (long)right.getVelocity(RADIANS));

            left.setVelocity(corL+leftV, RADIANS);
            right.setVelocity(corR+rightV, RADIANS);
        }

//        double[] radii = new double[1];
//        radii[0] = .3;
//        double[] arcLength = new double[1];
//        arcLength[0] = 0.1;
//        NeoPath trajectory3 = new SplinePath(0.3683, 0.3, 4, radii, arcLength);
//        trajectory3.build();
//        double velocity = 0.1;
//        double time = arcLength[0]/velocity;
//
//
//
//        ElapsedTime t = new ElapsedTime();
//        t.reset();
//        while(t.milliseconds() < time*1000){
//            double leftV = velocity-velocity*(0.3683/(2*radii[0]));
//            double rightV = velocity+velocity*(0.3683/(2*radii[0]));
//            leftV/=0.048;
//            rightV/=0.048;
//            leftV*=527.7;
//            rightV*=537.7;
//            telemetry.addData("lefAt driven velocity: ", leftV);
//            telemetry.addData("right driven velocity: ", rightV);
//            telemetry.update();
//            left.setVelocity(-1*leftV);
//            right.setVelocity(rightV);
//        }

//        while(!trajectory3.getCompleted()){
//            double leftV = convert(trajectory3.getLeftVelocity(t.milliseconds()/1000));
//            double rightV = convert(trajectory3.getRightVelocity(t.milliseconds()/1000));
//            telemetry.addData("left driven velocity: ", leftV);
//            telemetry.addData("right driven velocity: ", rightV);
//            telemetry.update();
//
//            left.setVelocity(-1*leftV);
//            right.setVelocity(rightV);
//        }
//        left.setVelocity(0);
//        right.setVelocity(0);
    }
}
