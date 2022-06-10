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
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PeripheralType;

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
     * Constructor for TwoWheelPathSequence to assign used objects
     * @param paths is the ArrayList of paths
     * @param left is the left motor (presumed to be negative to go forward)
     * @param right is the right motor (presumed to be positive to go forward)
     * @param wheelR is the wheel's radius
     */
    public TwoWheelPathSequence(ArrayList<NeoPath> paths, DcMotorEx left, DcMotorEx right, double wheelR){
        trajectory = paths;
        wheelRadius = wheelR;

        this.left = left;
        this.right = right;

    }


    /**
     * Actually moves the robot along the specified NeoPaths.
     * @Precondition the motor and trajectory objects have been created
     * @Postcondition the path has been followed
     */
    public final void follow(){
        if(left == null || right == null || trajectory == null)
            throw new InternalError("Null object parameter passed to TwoWheelPathSequence (in TwoWheelPathSequence.follow())");

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

                k3 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
                pid3 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

                k4 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
                pid4 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            }
            else if(p.getType() == PathType.TURN){
                k3 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_TURN);
                pid3 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_TURN);

                k4 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_TURN);
                pid4 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_TURN);
            }
            else{
                k3 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
                pid3 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

                k4 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
                pid4 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
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
