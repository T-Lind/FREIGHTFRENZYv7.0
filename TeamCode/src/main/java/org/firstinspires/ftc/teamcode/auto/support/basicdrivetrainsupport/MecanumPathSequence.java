package org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.MecLine;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.MecPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Path;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PIDController;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequenceFather;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

import java.util.ArrayList;

/**
 * Program to take linear velocities from each wheel and translate
 * them into mecanum 4wd
 * Created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */
public class MecanumPathSequence extends PathSequenceFather {
    /**
     * Motor object - first left motor
     */
    private DcMotorEx leftFront;

    /**
     * Motor object - second left motor
     */
    private DcMotorEx leftBack;

    /**
     * Motor object - first right motor
     */
    private DcMotorEx rightFront;

    /**
     * Motor object - second right motor
     */
    private DcMotorEx rightBack;

    /**
     * Constructor for FourWheelPathSequence, assigns used objects
     * @param paths is the ArrayList of paths
     * @param leftFront and is a left motor (presumed to be negative to go forward) does not matter which
     * @param leftBack and is a left motor (presumed to be negative to go forward) does not matter which
     * @param rightFront is the right motor (presumed to be positive to go forward) does not matter which
     * @param rightBack is the right motor (presumed to be positive to go forward) does not matter which
     * @param wheelR is the wheel's radius
     */
    public MecanumPathSequence(ArrayList<Path> paths, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double wheelR){
        trajectory = paths;
        wheelRadius = wheelR;

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack = rightBack;
    }

    /**
     * Actually moves the robot along the specified Paths.
     * Precondition:  the motor and trajectory objects have been created
     * Postcondition: the path has been followed
     */
    public final void follow(){
        if(leftFront == null || leftBack == null || rightFront == null || rightBack == null || trajectory == null)
            throw new RuntimeException("Null object parameter pssed to FourWheelPathSequence (in FourWheelPathSequence.follow())");

        ElapsedTime t = new ElapsedTime();
        t.reset();
        for(Path p : trajectory){
            if(!p.getBuilt())
                p.build();

            // Create kalman filter and PID objects
            KalmanFilter kLeft1 = new KalmanFilter();
            PIDController pidLeft1 = new PIDController();

            KalmanFilter kLeft2 = new KalmanFilter();
            PIDController pidLeft2 = new PIDController();

            KalmanFilter kRight1 = new KalmanFilter();
            PIDController pidRight1 = new PIDController();

            KalmanFilter kRight2 = new KalmanFilter();
            PIDController pidRight2 = new PIDController();

            double offset = t.milliseconds();

            // Execute the path
            while(!p.getCompleted()){
                if(p.getType() != PathType.MECLINE) {
                    // Get the velocities from what the path says the end result velocities should be
                    double leftV = Path.convertForStandardDrivetrain(wheelRadius, p.getLeftVelocity((t.milliseconds() - offset) / 1000));
                    double rightV = Path.convertForStandardDrivetrain(wheelRadius, p.getRightVelocity((t.milliseconds() - offset) / 1000));

                    // Correct based on PID and kalman filter
                    double corLF = pidLeft1.update((long) leftV, (long) kLeft1.filter(leftFront.getVelocity(RADIANS)));
                    double corLB = pidLeft2.update((long) leftV, (long) kLeft2.filter(leftBack.getVelocity(RADIANS)));
                    double corRF = pidRight1.update((long) rightV, (long) kRight1.filter(rightFront.getVelocity(RADIANS)));
                    double corRB = pidRight2.update((long) rightV, (long) kRight2.filter(rightBack.getVelocity(RADIANS)));

                    // Write the corrected values
                    leftFront.setVelocity(corLF + leftV, RADIANS);
                    leftBack.setVelocity(corLB + leftV, RADIANS);

                    rightFront.setVelocity(corRF + rightV, RADIANS);
                    rightBack.setVelocity(corRB + rightV, RADIANS);
                }

                else {
                    // Create kalman filter and PID objects
                    double leftFrontV = Path.convertForStandardDrivetrain(wheelRadius,
                            ((MecPath)p).getLeftFrontVelocity((t.milliseconds() - offset) / 1000));
                    double leftBackV = Path.convertForStandardDrivetrain(wheelRadius,
                            ((MecPath)p).getLeftBackVelocity((t.milliseconds() - offset) / 1000));
                    double rightFrontV = Path.convertForStandardDrivetrain(wheelRadius,
                            ((MecPath)p).getRightFrontVelocity((t.milliseconds() - offset) / 1000));
                    double rightBackV = Path.convertForStandardDrivetrain(wheelRadius,
                            ((MecPath)p).getRightBackVelocity((t.milliseconds() - offset) / 1000));

                    double corLF = pidLeft1.update((long) leftFrontV, (long) kLeft1.filter(leftFront.getVelocity(RADIANS)));
                    double corLB = pidLeft2.update((long) leftBackV, (long) kLeft2.filter(leftBack.getVelocity(RADIANS)));
                    double corRF = pidRight1.update((long) rightFrontV, (long) kRight1.filter(rightFront.getVelocity(RADIANS)));
                    double corRB = pidRight2.update((long) rightBackV, (long) kRight2.filter(rightBack.getVelocity(RADIANS)));

                    // Write the corrected values
                    leftFront.setVelocity(corLF + leftFrontV, RADIANS);
                    leftBack.setVelocity(corLB + leftBackV, RADIANS);

                    rightFront.setVelocity(corRF + rightFrontV, RADIANS);
                    rightBack.setVelocity(corRB + rightBackV, RADIANS);
                }
            }
            resetPaths();
        }
    }
}
