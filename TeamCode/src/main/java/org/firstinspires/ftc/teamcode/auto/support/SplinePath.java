package org.firstinspires.ftc.teamcode.auto.support;
/**
 * Translates a series of circle radii/arc lengths into motor power values for a 2 wheeled robot.
 * created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
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
    private boolean reversed;
    private ArrayList<Double> times;

    // track width is how far the wheels are apart, r is the radius of each of the turns, v is an ArrayList of static arrays of the velocities.

    /**
     * Constructor.
     * Precondition: each static array in v is 3 long, size of r and v is equal
     * Convention: positive arc length is CCW, negative is CW
     * @param tw track width (m)
     * @param v maximum linear velocity (m/s)
     * @param accTime acceleration and deceleration time (s)
     * @param r is the radii of the arcs in the spline
     * @param al are the arc lengths travelled in each corresponding radii.
     */
    public SplinePath(double tw, double v, double accTime, double[] r, double[] al){
        trackWidth = tw;
        radii = r;
        arcLengths = al;
        velocity = v;
        accelerationTime = accTime;
        TpA = 0;
        TpD = 0;

        times = new ArrayList<Double>();
        reversed = false;
        setType("Spline");
    }
    /**
     * Constructor.
     * Each static array in v is 3 long, size of r and v is equal
     * Convention: positive arc length is CCW, negative is CW
     * @param tw track width (m)
     * @param v maximum linear velocity (m/s)
     * @param accTime acceleration and deceleration time (s)
     * @param r is the radii of the arcs in the spline
     * @param al are the arc lengths travelled in each corresponding radii.
     * @param reversed is if the path should be followed backwards.
     */
    public SplinePath(double tw, double v, double accTime, double[] r, double[] al, boolean reversed){
        trackWidth = tw;
        radii = r;
        arcLengths = al;
        velocity = v;
        accelerationTime = accTime;
        TpA = 0;
        TpD = 0;

        times = new ArrayList<Double>();
        this.reversed = reversed;
        setType("Spline");
    }

    /**
     * Compute the various aspects of the spline like ramp-up and ramp-down appending times, the times ArrayList, and ArcLengths list.
     */
    @Override
    public void build(){
        for(int i=1;i<arcLengths.length-1;i++)
            arcLengths[i]*=(2*3.14159265);
        TpA = (3*Math.abs(arcLengths[0])-2*velocity*accelerationTime)/(3*velocity);
        TpD = (Math.abs(arcLengths[arcLengths.length-1])-((velocity*accelerationTime)/3))/velocity;
        System.out.println("TpA, TpD: "+TpA+" "+TpD);
        times.add(accelerationTime+TpA);
        for(int i=1;i<arcLengths.length-1;i++)
            times.add(times.get(times.size()-1)+Math.abs(arcLengths[i])*velocity);
        times.add(times.get(times.size()-1)+accelerationTime+TpD);
        setBuilt(true);
    }

    /**
     * Compute the execute time
     * @return return the execute time
     */
    @Override
    public double getExecuteTime(){
        return times.get(times.size()-1);
    }

    
     /**
     * Get the times ArrayList - not used except for debugging, times is the time at which every sequential part of the spline executes
     * @return the times ArrayList
     */
    public ArrayList<Double> getTimes(){
        return times;
    }

    /**
    * Gets the current arc that is executing based on the current time
    * @param t is the current time since the start of the spline
    */
    public int getArc(double t){
        for(int i=0;i<times.size();i++)
            if(t < times.get(i))
                return i;
        return -1;
    }
    
    /**
    * Get the linear velocity of the WHOLE robot based off of the current time. It's only != to the velocity specified in the constructor in the ramp-up and ramp-down times.
    * @param t is the current time into the spline
    */
    public double getVelocity(double t){
        int arc = getArc(t);
        if(arc == -1)
            return 0;

        if(arc > 0 && arc < arcLengths.length-1)
            return velocity;
        else if(arc == 0){
            if(t < accelerationTime)
                return velocity*Math.sqrt(t/accelerationTime);
            return velocity;
        }
        else {
            if(t < times.get(times.size()-2)+TpD)
                return velocity;
            return velocity-velocity*Math.sqrt((t-times.get(times.size()-2)-TpD)/accelerationTime);
        }
    }
    
    /**
    * Get the left linear velocity based on the current time, accounting for negative arc lengths, reversed boolean, and different arc radii.
    * @param t is the current time into the spline.
    * @return the left wheel's linear velocity.
    */
    @Override
    public double getLeftVelocity(double t){
        if(!reversed) {
            if (getArc(t) == -1) {
                this.setCompleted(true);
                return 0;
            }
            double v = getVelocity(t);
            if (arcLengths[getArc(t)] > 0)
                v = v - (v * trackWidth / (2 * radii[getArc(t)]));
            else
                v = v + (v * trackWidth / (2 * radii[getArc(t)]));
            v *= -1;
            return v;
        } else {
            if (getArc(t) == -1) {
                this.setCompleted(true);
                return 0;
            }
            double v = getVelocity(t);
            if (arcLengths[getArc(t)] > 0)
                v = v - (v * trackWidth / (2 * radii[getArc(t)]));
            else
                v = v + (v * trackWidth / (2 * radii[getArc(t)]));
            return v;
        }
    }
    

    /**
    * Get the right linear velocity based on the current time, accounting for negative arc lengths, reversed boolean, and different arc radii.
    * @param t is the current time into the spline.
    * @return the right wheel's linear velocity.
    */    
    @Override
    public double getRightVelocity(double t){
        if(!reversed) {
            if (getArc(t) == -1) {
                this.setCompleted(true);
                return 0;
            }
            double v = getVelocity(t);
            if (arcLengths[getArc(t)] > 0)
                return v + (v * trackWidth / (2 * radii[getArc(t)]));
            return v - (v * trackWidth / (2 * radii[getArc(t)]));
        } else {
            if (getArc(t) == -1) {
                this.setCompleted(true);
                return 0;
            }
            double v = getVelocity(t);
            if (arcLengths[getArc(t)] > 0){
                v = v + (v * trackWidth / (2 * radii[getArc(t)]));
                v *= -1;
                return v;
            }
            v = v - (v * trackWidth / (2 * radii[getArc(t)]));
            return -1*v;
        }
    }

}

