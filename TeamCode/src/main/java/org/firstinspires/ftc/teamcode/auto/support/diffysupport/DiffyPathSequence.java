package org.firstinspires.ftc.teamcode.auto.support.diffysupport;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PIDController;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequenceFather;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PeripheralType;

import java.util.ArrayList;

/**
 * Program to take linear velocities from each wheel and translate
 * them into 4wd
 * Created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */
public class DiffyPathSequence extends PathSequenceFather {

    // Motor variables specific to a diffy swerve
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    // Conversion number for diffy swerves
    private final double DIFFY_FACTOR = 100;

    /**
     * Constructor for DiffyPathSequence.
     * @param d is the ArrayList of paths
     * @param leftFront is the front left motor - CHUB side is the front
     * @param leftBack is the back left motor - CHUB side is the front
     * @param rightFront is the front right motor - CHUB side is the front
     * @param rightBack is the back right motor - CHUB side is the front
     * @param wheelR is the wheel's radius
     */
    public DiffyPathSequence(ArrayList<NeoPath> d, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double wheelR){
        trajectory = d;
        wheelRadius = wheelR;

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack= rightBack;

    }

    /**
     * Convert a distance in meters (or a velocity) to radians
     * @param meters the distance or velocity in meters
     * @return the distance or velocity in radians
     */
    private double convertDistance(double meters){
        return meters*(18.1*meters+9.61);
    }


    /**
     * Actually moves the robot along the specified NeoPaths.
     * Also adheres to InsertMarkers if any.
     * NOTE: cannot rotate diffy swerve pod angles at this time
     * @Precondition motor objects are not null and have been instantiated
     * @Postcondition every path has been executed
     */
    @Override
    public final void follow(){
        if(!(leftFront != null && leftBack != null && rightFront != null && rightBack != null))
            throw new InternalError("All motor objects in DiffyPathSequence.follow() must not be null!");
//        assert leftFront != null && leftBack != null && rightFront != null && rightBack != null : "All motor objects in DiffyPathSequence.follow() must not be null!";
//        assert trajectory != null: "The trajectory in DiffyPathSequence.follow() cannot be null!";

        ElapsedTime t = new ElapsedTime();
        t.reset();

        // Go through every path in the trajectory
        for(NeoPath path : trajectory){
            // Make sure the path is built
            if(!path.getBuilt())
                path.build();

            // Create kalman filter and PID objects
            KalmanFilter kLeft1 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidLeft1 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            KalmanFilter kLeft2 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidLeft2 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            KalmanFilter kRight1 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidRight1 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);

            KalmanFilter kRight2 = new KalmanFilter(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);
            PIDController pidRight2 = new PIDController(PeripheralType.DRIVETRAIN_MOTOR_STRAIGHT);


            // Used to only consider the time into this individual path
            double offset = t.milliseconds();

            // Execute the path
            while(!path.getCompleted()){
                // Get the velocities from what the path says the end result velocities should be
                double leftV = path.getLeftVelocity((t.milliseconds()-offset)/1000);
                double rightV = path.getRightVelocity((t.milliseconds()-offset)/1000);

                // Convert the velocities into pod velocities
                double leftFrontTargetV = -convertDistance(leftV);
                double leftBackTargetV = convertDistance(leftV);
                double rightFrontTargetV = convertDistance(rightV);
                double rightBackTargetV = -convertDistance(leftV);

                // Correct using PID loop and Kalman Filter
                double corL1 = pidLeft1.update((long)leftFrontTargetV, (long)kLeft1.filter(leftFront.getVelocity(RADIANS)));
                double corL2 = pidLeft2.update((long)leftBackTargetV, (long)kLeft2.filter(leftBack.getVelocity(RADIANS)));
                double corR1 = pidRight1.update((long)rightFrontTargetV, (long)kRight1.filter(rightFront.getVelocity(RADIANS)));
                double corR2 = pidRight2.update((long)rightBackTargetV, (long)kRight2.filter(rightBack.getVelocity(RADIANS)));

                // Write the corrected velocities to the motors
                leftFront.setVelocity(leftFrontTargetV+corL1, RADIANS);
                leftBack.setVelocity(leftBackTargetV+corL2, RADIANS);
                rightFront.setVelocity(rightFrontTargetV+corR1, RADIANS);
                rightBack.setVelocity(rightBackTargetV+corR2, RADIANS);


            }
            // Set each path back to an unused state so the trajectory could be run again
            resetPaths();
        }
    }
}
