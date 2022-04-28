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
    private double accelerationTime;
    private double TpA;
    private double TpD;

    private ArrayList<Double> times;

    // track width is how far the wheels are apart, r is the radius of each of the turns, v is an ArrayList of static arrays of the velocities.
    /**
     * @Precondition each static array in v is 3 long, size of r and v is equal
     * @Convention positive arc length is CCW, negative is CW
     * **/
    public SplinePath(double tw, double v, double accTime, double[] r, double[] al){
        trackWidth = tw;
        radii = r;
        arcLengths = al;
        velocity = v;
        accelerationTime = accTime;
        TpA = 0;
        TpD = 0;

        times = new ArrayList<Double>();
    }

    @Override
    public void build(){
        TpA = (3*arcLengths[0]-2*velocity*accelerationTime)/(3*velocity);
        TpD = (3*arcLengths[arcLengths.length-1]-velocity*accelerationTime)/(3*velocity);
        double buildTime = TpA+TpD+2*accelerationTime;

        for(int i=1;i< radii.length-1;i++) {
            times.add(buildTime);
            buildTime += (arcLengths[i] * radii[i] / velocity);

        }

        super.setExecuteTime(buildTime);
    }

    public double getVelocity(double t){
        if(t > this.getExecuteTime())
            this.setCompleted(true);

        if(t < accelerationTime){
            return velocity*Math.sqrt(t/accelerationTime);
        }
        if(t < this.getExecuteTime()-TpD){
            return velocity;
        }
        return velocity-(velocity*Math.sqrt(t/accelerationTime));
    }

    public double getRadii(double t){
        for(int i=0;i<times.size();i++){
            if(t < times.get(i))
                return radii[i];
        }

        return 1E-4;
    }

    @Override
    public double getLeftVelocity(double t){
        double v = -1*getVelocity(t);
        return v - v*(trackWidth/(2*getRadii(t)));

    }
    @Override
    public double getRightVelocity(double t){
        double v = getVelocity(t);
        return v + v*(trackWidth/(2*getRadii(t)));

    }
}
