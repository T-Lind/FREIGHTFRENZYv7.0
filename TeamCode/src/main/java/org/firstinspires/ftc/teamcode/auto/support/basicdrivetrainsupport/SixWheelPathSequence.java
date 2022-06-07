package org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport;
/**
 * Program to take linear velocities from each wheel and translate
 * them into 6wd
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
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoMarkerList;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PIDController;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequenceFather;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.RunnableCollective;

import java.util.ArrayList;

public class SixWheelPathSequence extends PathSequenceFather {
    private DcMotorEx left1;
    private DcMotorEx left2;
    private DcMotorEx left3;
    private DcMotorEx right1;
    private DcMotorEx right2;
    private DcMotorEx right3;

    /**
     *
     * @param paths is the ArrayList of paths
     * @param left1 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param left2 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param left3 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param right1 is the right motor (presumed to be positive to go forward) does not matter which
     * @param right2 is the right motor (presumed to be positive to go forward) does not matter which
     * @param right3 is the right motor (presumed to be positive to go forward) does not matter which
     * @param wheelR is the wheel's radius
     *
     * @Precondition the left and right motors are objects that have been externally created
     */
    public SixWheelPathSequence(ArrayList<NeoPath> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx left3, DcMotorEx right1, DcMotorEx right2, DcMotorEx right3, double wheelR){
        trajectory = paths;
        wheelRadius = wheelR;

        this.left1 = left1;
        this.left2 = left1;
        this.left3 = left1;
        this.right1 = right1;
        this.right2= right2;
        this.right3= right3;

        markerList = null;
        runnableCollectiveMarkerList = null;
    }
    /**
     *
     * @param paths is the ArrayList of paths
     * @param left1 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param left2 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param left3 and is a left motor (presumed to be negative to go forward) does not matter which
     * @param right1 is the right motor (presumed to be positive to go forward) does not matter which
     * @param right2 is the right motor (presumed to be positive to go forward) does not matter which
     * @param right3 is the right motor (presumed to be positive to go forward) does not matter which
     * @param wheelR is the wheel's radius
     * @param m is the MarkerList
     *
     * @Precondition the left and right motors are objects that have been externally created
     */
    public SixWheelPathSequence(ArrayList<NeoPath> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx left3, DcMotorEx right1, DcMotorEx right2, DcMotorEx right3, double wheelR, NeoMarkerList m){
        trajectory = paths;
        wheelRadius = wheelR;

        this.left1 = left1;
        this.left2 = left2;
        this.left3 = left3;
        this.right1 = right1;
        this.right2= right2;
        this.right3= right3;

        markerList = m;
        runnableCollectiveMarkerList = new RunnableCollective(markerList);
    }

    /**
     * Actually moves the robot along the specified NeoPaths.
     * Also adheres to InsertMarkers if any.
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

            KalmanFilter kLeft1 = new KalmanFilter(0);
            PIDController pidLeft1 = new PIDController(0);

            KalmanFilter kLeft2 = new KalmanFilter(0);
            PIDController pidLeft2 = new PIDController(0);

            KalmanFilter kLeft3 = new KalmanFilter(0);
            PIDController pidLeft3 = new PIDController(0);

            KalmanFilter kRight1 = new KalmanFilter(0);
            PIDController pidRight1 = new PIDController(0);

            KalmanFilter kRight2 = new KalmanFilter(0);
            PIDController pidRight2 = new PIDController(0);

            KalmanFilter kRight3 = new KalmanFilter(0);
            PIDController pidRight3 = new PIDController(0);

            double offset = t.milliseconds();

            // Execute the path
            while(!p.getCompleted()){
                // Get the velocities from what the path says the end result velocities should be
                double leftV = NeoPath.convert(wheelRadius, p.getLeftVelocity((t.milliseconds()-offset)/1000));
                double rightV = NeoPath.convert(wheelRadius, p.getRightVelocity((t.milliseconds()-offset)/1000));

                // Correct based on PID and kalman filter
                double corL1 = pidLeft1.update((long)leftV, (long)kLeft1.filter(left1.getVelocity(RADIANS)));
                double corL2 = pidLeft2.update((long)leftV, (long)kLeft2.filter(left2.getVelocity(RADIANS)));
                double corL3 = pidLeft3.update((long)leftV, (long)kLeft3.filter(left3.getVelocity(RADIANS)));
                double corR1 = pidRight1.update((long)rightV, (long)kRight1.filter(right1.getVelocity(RADIANS)));
                double corR2 = pidRight2.update((long)rightV, (long)kRight2.filter(right2.getVelocity(RADIANS)));
                double corR3 = pidRight3.update((long)rightV, (long)kRight3.filter(right3.getVelocity(RADIANS)));

                // Write the corrected values
                left1.setVelocity(corL1+leftV, RADIANS);
                left2.setVelocity(corL2+leftV, RADIANS);
                left3.setVelocity(corL3+leftV, RADIANS);

                right1.setVelocity(corR1+rightV, RADIANS);
                right2.setVelocity(corR2+rightV, RADIANS);
                right3.setVelocity(corR3+rightV, RADIANS);
            }
            resetPaths();
        }
        if(runnableCollectiveMarkerList != null)
            runnableCollectiveMarkerList.setStopMarkers(true);
    }
}
