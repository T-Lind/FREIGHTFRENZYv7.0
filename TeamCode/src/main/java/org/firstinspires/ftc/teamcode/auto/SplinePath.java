package org.firstinspires.ftc.teamcode.auto;
/**
 * Translates a series of circle radii/arc lengths into motor power values for a 2 wheeled robot.
 * created by @author Tiernan Lindauer for FTC team 7797.
 * @license  Creative Commons
 * Last edited 4/18/22
 *
 */

import java.util.ArrayList;


public class SplinePath extends NeoPath {
    private double[] radii;
    private double[] arcLengths;
    private double velocity;
    private double trackWidth;

    private ArrayList<double[]> leftVel;
    private ArrayList<double[]> rightVel;


    // track width is how far the wheels are apart, r is the radius of each of the turns, v is an ArrayList of static arrays of the velocities.
    /**
     * @Precondition each static array in v is 3 long, size of r and v is equal
     * @Convention positive arc length is CCW, negative is CW
     * **/
    public SplinePath(double tw, double v, double[] r, double[] al){
        trackWidth = tw;
        radii = r;
        arcLengths = al;
        velocity = v;

        leftVel = new ArrayList<double[]>();
        rightVel = new ArrayList<double[]>();
    }

    @Override
    public void build(){
        double buildTime = 0;
        for(int i=0;i<radii.length;i++){
            double time = Math.abs(arcLengths[i]*radii[i]/velocity);
            double omega = velocity/radii[i];
            double difference = trackWidth*omega/2;

            if(arcLengths[i] > 0) {
                // format is velocity and then start time
                double[] leftV = {velocity - difference, time + buildTime};
                leftVel.add(leftV);

                double[] rightV = {velocity + difference, time + buildTime};
                rightVel.add(rightV);
            }
            else{
                // format is velocity and then start time
                double[] leftV = {velocity + difference, time + buildTime};
                leftVel.add(leftV);

                double[] rightV = {velocity - difference, time + buildTime};
                rightVel.add(rightV);
            }
            buildTime += time;
        }
        super.setExecuteTime(buildTime);
    }

    @Override
    public double getLeftVelocity(double t){
        for (double[] doubles : leftVel) {
            if (doubles[1] > t)
                return doubles[0];
        }
        return 0;
    }
    @Override
    public double getRightVelocity(double t){
        for (double[] doubles : rightVel) {
            if (doubles[1] > t)
                return doubles[0];
        }
        return 0;
    }
}
