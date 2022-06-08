package org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import java.util.ArrayList;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PIDController;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequenceFather;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Program to take linear velocities from each wheel and translate
 * them into 2wd
 * Created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */
public class TwoWheelPathSequence extends PathSequenceFather{
    private DcMotorEx left;
    private DcMotorEx right;
    /**
     *
     * @param d is the ArrayList of paths
     * @param left is the left motor (presumed to be negative to go forward)
     * @param right is the right motor (presumed to be positive to go forward)
     * @param wheelR is the wheel's radius
     *
     * @Precondition the left and right motors are objects that have been externally created
     */
    public TwoWheelPathSequence(ArrayList<NeoPath> d, DcMotorEx left, DcMotorEx right, double wheelR){
        trajectory = d;
        wheelRadius = wheelR;

        this.left = left;
        this.right = right;

    }


    /**
     * Actually moves the robot along the specified NeoPaths.
     * Also adheres to InsertMarkers if any.
     */
    public final void follow(){
        ElapsedTime t = new ElapsedTime();
        t.reset();

        for(NeoPath p : trajectory){
            if(!p.getBuilt())
                p.build();

            KalmanFilter k3;
            PIDController pid3;
            KalmanFilter k4;
            PIDController pid4;

            // Experimental bit here
            if(p.getType() == PathType.LINE || p.getType() == PathType.SPLINE){

                k3 = new KalmanFilter(0);
                pid3 = new PIDController(0);

                k4 = new KalmanFilter(0);
                pid4 = new PIDController(0);
            }
            else if(p.getType() == PathType.TURN){
                k3 = new KalmanFilter(2);
                pid3 = new PIDController(2);

                k4 = new KalmanFilter(2);
                pid4 = new PIDController(2);
            }
            else{
                k3 = new KalmanFilter(0);
                pid3 = new PIDController(0);

                k4 = new KalmanFilter(0);
                pid4 = new PIDController(0);
            }

            double offset = t.milliseconds();
            while(!p.getCompleted()){
                double leftV = NeoPath.convert(wheelRadius, p.getLeftVelocity((t.milliseconds()-offset)/1000));
                double rightV = NeoPath.convert(wheelRadius, p.getRightVelocity((t.milliseconds()-offset)/1000));

                double corL = pid3.update((long)leftV, (long)k3.filter(left.getVelocity(RADIANS)));
                double corR = pid4.update((long)rightV, (long)k4.filter(right.getVelocity(RADIANS)));

                left.setVelocity(corL+leftV, RADIANS);
                right.setVelocity(corR+rightV, RADIANS);
            }

            resetPaths();
        }
    }

}
