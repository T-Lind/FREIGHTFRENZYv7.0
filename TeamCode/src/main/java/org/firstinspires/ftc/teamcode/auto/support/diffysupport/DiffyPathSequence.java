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

import org.firstinspires.ftc.teamcode.auto.support.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.KalmanFilter;
import org.firstinspires.ftc.teamcode.auto.support.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.PIDController;

import java.util.ArrayList;

public class DiffyPathSequence {

    private ArrayList<NeoPath> trajectory;
    private double wheelRadius;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;


    MarkerList markerList;
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
    }
    /**
     *
     * @param d is the ArrayList of paths
     * @param leftFront is the front left motor - CHUB side is the front
     * @param leftBack is the back left motor - CHUB side is the front
     * @param rightFront is the front right motor - CHUB side is the front
     * @param rightBack is the back right motor - CHUB side is the front
     * @param wheelR is the wheel's radius
     * @param m is the marker list
     *
     * Precondition: the left and right motors are objects that have been externally created
     */
    public DiffyPathSequence(ArrayList<NeoPath> d, DcMotorEx leftFront, DcMotorEx leftBack, DcMotorEx rightFront, DcMotorEx rightBack, double wheelR, MarkerList m){
        trajectory = d;
        wheelRadius = wheelR;

        this.leftFront = leftFront;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.rightBack= rightBack;

        markerList = m;
    }

    private double convertDistance(double meters){
        return meters*(18.1*meters+9.61);
    }

    /**
     * Build each NeoPath in the trajectory.
     */
    public void buildAll(){
        for(NeoPath p : trajectory)
            p.build();
    }

    /**
     * Build a specific trajectory
     * @param i the index of the NeoPath that you want to build.
     */
    public void build(int i){
        trajectory.get(i).build();
    }

    /**
     * Precondition:
     * all paths have been build using the buildAll() method.
     * Note that this is not technically necessary but reduces lag time.
     */

    public void reset(){
        for(NeoPath p : trajectory)
            p.setCompleted(false);
    }

    /**
     * Actually moves the robot along the specified NeoPaths.
     * Also adheres to InsertMarkers if any.
     * NOTE: cannot rotate diffy swerve pod angles at this time
     */
    public void follow(){
        ElapsedTime t = new ElapsedTime();
        t.reset();
        for(NeoPath p : trajectory){
            if(!p.getBuilt())
                p.build();

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
                double leftV = convertDistance(p.getLeftVelocity((t.milliseconds()-offset)/1000));
                double rightV = convertDistance(p.getRightVelocity((t.milliseconds()-offset)/1000));

                double leftFrontTargetV = -convertDistance(leftV);
                double leftBackTargetV = convertDistance(leftV);
                double rightFrontTargetV = convertDistance(rightV);
                double rightBackTargetV = -convertDistance(leftV);

                double corL1 = pidLeft1.update((long)leftFrontTargetV, (long)kLeft1.filter(leftFront.getVelocity(RADIANS)));
                double corL2 = pidLeft2.update((long)leftBackTargetV, (long)kLeft2.filter(leftBack.getVelocity(RADIANS)));
                double corR1 = pidRight1.update((long)rightFrontTargetV, (long)kRight1.filter(rightFront.getVelocity(RADIANS)));
                double corR2 = pidRight2.update((long)rightBackTargetV, (long)kRight2.filter(rightBack.getVelocity(RADIANS)));

                leftFront.setVelocity(leftFrontTargetV+corL1);
                leftBack.setVelocity(leftBackTargetV+corL2);
                rightFront.setVelocity(rightFrontTargetV+corR1);
                rightBack.setVelocity(rightBackTargetV+corR2);

                if(markerList != null)
                    for(InsertMarker m : markerList.getMarkers())
                        m.execute(t.milliseconds()/1000);
            }
            reset();
        }
    }
}
