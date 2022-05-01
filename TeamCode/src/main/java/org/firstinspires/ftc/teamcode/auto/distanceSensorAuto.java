package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.support.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.Line;
import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.PIDController;

import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name="distanceSensorAuto")
public class distanceSensorAuto extends LinearOpMode {
    private DcMotorEx left, right;
    DistanceSensor distanceSensor;

    protected double convert(double v){
        v/=0.048; // convert to angular velocity by radius
        v/=(2*3.14159);
        return v;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        left = (DcMotorEx) hardwareMap.dcMotor.get("L");
        right = (DcMotorEx) hardwareMap.dcMotor.get("R");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");

        KalmanFilter kD = new KalmanFilter(2, 10);

        double d = distanceSensor.getDistance(DistanceUnit.MM);
        while(!opModeIsActive()) {
            d = distanceSensor.getDistance(DistanceUnit.MM);
            d = kD.filter(d)/1000;
            telemetry.addData("Distance", d);
            telemetry.update();
        }
        KalmanFilter k = new KalmanFilter(7,4);
        PIDController pid = new PIDController(0.5,0.5,0.2);

        KalmanFilter k2 = new KalmanFilter(7,4);
        PIDController pid2 = new PIDController(0.5,0.5,0.2);

        NeoPath trajectory3 = new Line(d-0.1,0.4);
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


    }
}
