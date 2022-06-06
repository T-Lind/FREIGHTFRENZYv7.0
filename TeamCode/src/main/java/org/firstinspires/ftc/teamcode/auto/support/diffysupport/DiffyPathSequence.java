package org.firstinspires.ftc.teamcode.auto.support.diffysupport;
/**
 * Program to take linear velocities from each wheel and translate
 * them into 4wd
 * Created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PIDController;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequenceFather;

import java.util.ArrayList;

public class DiffyPathSequence implements PathSequenceFather {

    private ArrayList<NeoPath> trajectory;
    private double wheelRadius;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    private MarkerList markerList;

    // Calculate the reduction due to diffy swerve
    private final static double DIFFY_FACTOR = 2*(64/18.0)*(64/24.0);

    /**
     *
     * @param trajectory is the ArrayList of paths
     * @param leftFront is the front left motor - CHUB side is the front
     * @param leftBack is the back left motor - CHUB side is the front
     * @param rightFront is the front right motor - CHUB side is the front
     * @param rightBack is the back right motor - CHUB side is the front
     * @param wheelRadius is the wheel's radius
     *
     * @Precondition the left and right motors are objects that have been externally created
     */
    public DiffyPathSequence(ArrayList<NeoPath> trajectory, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double wheelRadius){
        assert wheelRadius > 0 : "Wheel radius must be greater than 0!";
        assert trajectory != null && leftFront != null && leftBack != null && rightFront != null && rightBack != null: "Input passed that is null!";

        this.trajectory = trajectory;
        this.wheelRadius = wheelRadius;

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack= rightBack;

        markerList = null;
    }
    /**
     *
     * @param trajectory is the ArrayList of paths
     * @param leftFront is the front left motor - CHUB side is the front
     * @param leftBack is the back left motor - CHUB side is the front
     * @param rightFront is the front right motor - CHUB side is the front
     * @param rightBack is the back right motor - CHUB side is the front
     * @param wheelRadius is the wheel's radius
     * @param markerList is the marker list
     *
     * Precondition: the left and right motors are objects that have been externally created
     */
    public DiffyPathSequence(ArrayList<NeoPath> trajectory, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double wheelRadius, MarkerList markerList){
        assert wheelRadius > 0 : "Wheel radius must be greater than 0!";
        assert trajectory != null && leftFront != null && leftBack != null && rightFront != null && rightBack != null && markerList != null: "Input passed that is null!";

        this.trajectory = trajectory;
        this.wheelRadius = wheelRadius;

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack= rightBack;

        this.markerList = markerList;
    }

    /**
     * Build each NeoPath in the trajectory.
     * @Postcondition every element of trajectory is built
     */
    public void buildAll(){
        for(NeoPath p : trajectory)
            p.build();
    }

    /**
     * Build a specific trajectory
     * @Precondition i is a nonzero, non-negative integer.
     * @param i the index of the NeoPath that you want to build.
     * @Postcondition only index i in the Neopath has been built.
     */
    public void build(int i){
        assert i >= 0 : "The index parameter of DiffyPathSequence.build() must be a real index!";

        trajectory.get(i).build();
    }

    /**
     * @Precondition all paths have been build using the buildAll() method.
     * Note that this is not technically necessary but reduces lag time.
     * Sets the completed state of each path back to false to allow for path reusability
     * @Postcondition only the completed variable in each NeoPath in trajectory has been changed
     */
    public void resetPaths(){
        for(NeoPath p : trajectory)
            p.setCompleted(false);
    }

    /**
     * @Precondition every path in trajectory has been built
     * Actually moves the robot along the specified NeoPaths.
     * Also adheres to InsertMarkers if any.
     * NOTE: cannot rotate diffy swerve pod angles at this time
     * @Postcondition the robot follows each path accurately
     */
    public void follow(){
        // Keep track of time with ElapsedTime
        ElapsedTime t = new ElapsedTime();
        t.reset();

        // Build path if the path has not been built before
        for(NeoPath p : trajectory){
            if(!p.getBuilt())
                p.build();

            // Create Kalman filters and PID control loops for each motor
            KalmanFilter kLeft1 = new KalmanFilter(0);
            PIDController pidLeft1 = new PIDController(0);

            KalmanFilter kLeft2 = new KalmanFilter(0);
            PIDController pidLeft2 = new PIDController(0);

            KalmanFilter kRight1 = new KalmanFilter(0);
            PIDController pidRight1 = new PIDController(0);

            KalmanFilter kRight2 = new KalmanFilter(0);
            PIDController pidRight2 = new PIDController(0);


            double offset = t.milliseconds();

            while(!p.getCompleted()){
                // Determine what each pod velocity should be
                double leftV = NeoPath.convert(wheelRadius, p.getLeftVelocity((t.milliseconds()-offset)/1000));
                double rightV = NeoPath.convert(wheelRadius, p.getRightVelocity((t.milliseconds()-offset)/1000));

                // Determine what each motor velocity should be
                double leftFrontTargetV = leftV*DIFFY_FACTOR;
                double leftBackTargetV = -leftV*DIFFY_FACTOR;
                double rightFrontTargetV = -rightV*DIFFY_FACTOR;
                double rightBackTargetV = rightV*DIFFY_FACTOR;

                // Correct using PID loop and Kalman Filter
                double corL1 = pidLeft1.update((long)leftFrontTargetV, (long)kLeft1.filter(leftFront.getVelocity(RADIANS)));
                double corL2 = pidLeft2.update((long)leftBackTargetV, (long)kLeft2.filter(leftBack.getVelocity(RADIANS)));
                double corR1 = pidRight1.update((long)rightFrontTargetV, (long)kRight1.filter(rightFront.getVelocity(RADIANS)));
                double corR2 = pidRight2.update((long)rightBackTargetV, (long)kRight2.filter(rightBack.getVelocity(RADIANS)));

                // Write the corrected values
                leftFront.setVelocity(leftFrontTargetV+corL1);
                leftBack.setVelocity(leftBackTargetV+corL2);
                rightFront.setVelocity(rightFrontTargetV+corR1);
                rightBack.setVelocity(rightBackTargetV+corR2);

                // Execute each InsertMarker in markerList
                if(markerList != null)
                    for(InsertMarker m : markerList.getMarkers())
                        m.execute(t.milliseconds()/1000);
            }
            // Set each path back to an unused state so the trajectory could be run again
            resetPaths();
        }
    }
}
