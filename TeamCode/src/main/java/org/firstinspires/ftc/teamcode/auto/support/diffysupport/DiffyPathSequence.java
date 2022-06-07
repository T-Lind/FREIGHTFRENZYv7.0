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
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoMarkerList;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PIDController;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequenceFather;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.RunnableCollective;

import java.util.ArrayList;

public class DiffyPathSequence extends PathSequenceFather {

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    private final double DIFFY_FACTOR = 100;

    /**
     *
     * @param d is the ArrayList of paths
     * @param leftFront is the front left motor - CHUB side is the front
     * @param leftBack is the back left motor - CHUB side is the front
     * @param rightFront is the front right motor - CHUB side is the front
     * @param rightBack is the back right motor - CHUB side is the front
     * @param wheelR is the wheel's radius
     *
     * Precondition: the left and right motors are objects that have been externally created
     */
    public DiffyPathSequence(ArrayList<NeoPath> d, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double wheelR){
        trajectory = d;
        wheelRadius = wheelR;

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack= rightBack;

        markerList = null;
        runnableCollectiveMarkerList = null;
    }
    /**
     *
     * @param d is the ArrayList of paths
     * @param leftFront is the front left motor - CHUB side is the front
     * @param leftBack is the back left motor - CHUB side is the front
     * @param rightFront is the front right motor - CHUB side is the front
     * @param rightBack is the back right motor - CHUB side is the front
     * @param wheelR is the wheel's radius
     * @param markerList is the marker list
     *
     * Precondition: the left and right motors are objects that have been externally created
     */
    public DiffyPathSequence(ArrayList<NeoPath> d, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double wheelR, NeoMarkerList markerList){
        trajectory = d;
        wheelRadius = wheelR;

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack= rightBack;

        this.markerList = markerList;
        runnableCollectiveMarkerList = new RunnableCollective(markerList);
    }

    private double convertDistance(double meters){
        return meters*(18.1*meters+9.61);
    }


    /**
     * Actually moves the robot along the specified NeoPaths.
     * Also adheres to InsertMarkers if any.
     * NOTE: cannot rotate diffy swerve pod angles at this time
     */
    @Override
    public void follow(){
        ElapsedTime t = new ElapsedTime();
        t.reset();

        if(runnableCollectiveMarkerList != null)
            runnableCollectiveMarkerList.setRunMarkers(true);

        for(NeoPath p : trajectory){
            if(!p.getBuilt())
                p.build();

            // Create kalman filter and PID objects
            KalmanFilter kLeft1 = new KalmanFilter(0);
            PIDController pidLeft1 = new PIDController(0);

            KalmanFilter kLeft2 = new KalmanFilter(0);
            PIDController pidLeft2 = new PIDController(0);

            KalmanFilter kRight1 = new KalmanFilter(0);
            PIDController pidRight1 = new PIDController(0);

            KalmanFilter kRight2 = new KalmanFilter(0);
            PIDController pidRight2 = new PIDController(0);


            double offset = t.milliseconds();

            // Execute the path
            while(!p.getCompleted()){
                // Get the velocities from what the path says the end result velocities should be
                double leftV = p.getLeftVelocity((t.milliseconds()-offset)/1000);
                double rightV = p.getRightVelocity((t.milliseconds()-offset)/1000);

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

                // Write the corrected values
                leftFront.setVelocity(leftFrontTargetV+corL1);
                leftBack.setVelocity(leftBackTargetV+corL2);
                rightFront.setVelocity(rightFrontTargetV+corR1);
                rightBack.setVelocity(rightBackTargetV+corR2);


            }
            // Set each path back to an unused state so the trajectory could be run again
            resetPaths();
        }
        // Make the marker threads stop
        if(runnableCollectiveMarkerList != null)
            runnableCollectiveMarkerList.setStopMarkers(true);
    }
}
