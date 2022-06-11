package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Direction;

import java.util.ArrayList;

/**
 * Translates a series of circle radii/arc lengths into motor power values for a 2 wheeled robot.
 * created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */
public class SplinePath extends Path {
    // Variables to characterize the spline
    /**
     * List of radii to follow.
     */
    private double[] radii;
    /**
     * List of arc lengths to follow.
     */
    private double[] arcLengths;

    /**
     * Velocity to move at during the bulk of the spline.
     */
    private double velocity;
    /**
     *  Distance between the wheels.
     */
    private double trackWidth;
    /**
     * Time the robot should spend accelerating or decelerating.
     */
    private double accelerationTime;
    /**
     * Additional time needed to fulfill the first arc.
     */
    private double additionalpathonetime;
    /**
     *  Additional time needed to fufill the last arc.
     */
    private double additionalPathTwoTime;
    /**
     * Whether the robot should move forward or go in reverse.
     */
    private Direction moveWay;

    /**
     *  Times at which to execute each part of the spline.
     */
    private ArrayList<Double> times;

    // track width is how far the wheels are apart, r is the radius of each of the turns, v is an ArrayList of static arrays of the velocities.

    /**
     * Constructor for SplinePath.
     * Precondition:  : each static array in v is 3 long, size of r and v is equal
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
        additionalpathonetime = 0;
        additionalPathTwoTime = 0;

        times = new ArrayList<>();
        moveWay = Direction.FORWARD;
    }
    /**
     * Constructor for SplinePath with reversed boolean.
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
        additionalpathonetime = 0;
        additionalPathTwoTime = 0;

        times = new ArrayList<>();

        if(reversed)
            moveWay = Direction.REVERSE;
        else
            moveWay = Direction.FORWARD;
    }

    /**
     * Compute the various aspects of the spline like ramp-up and ramp-down appending times, the times ArrayList, and ArcLengths list.
     * Precondition:  arcLengths and times is not null and has been instantiated
     * Postcondition: times have been successfully computed and this path has been built,
     */
    @Override
    public void build(){
        if(arcLengths == null || times == null )
            throw new RuntimeException("ArcLengths and times must be not null in SplinePath.build()");

        for(int i=1;i<arcLengths.length-1;i++)
            arcLengths[i]*=(2*3.14159265);
        additionalpathonetime = (3*Math.abs(arcLengths[0])-2*velocity*accelerationTime)/(3*velocity);
        additionalPathTwoTime = (Math.abs(arcLengths[arcLengths.length-1])-((velocity*accelerationTime)/3))/velocity;
        System.out.println("TpA, TpD: "+ additionalpathonetime +" "+ additionalPathTwoTime);
        times.add(accelerationTime+ additionalpathonetime);
        for(int i=1;i<arcLengths.length-1;i++)
            times.add(times.get(times.size()-1)+Math.abs(arcLengths[i])*velocity);
        times.add(times.get(times.size()-1)+accelerationTime+ additionalPathTwoTime);
        setBuilt(true);
    }

    /**
     * Compute the execute time
     * @return return the execute time
     * Precondition:  times has been instantiated and is not null
     * Postcondition: the appropriate execute time has been returned
     */
    @Override
    public double getExecuteTime(){
        if(times == null)
            throw new RuntimeException("Times must not be equal to null in SplinePath.getExecuteTime()");

        return times.get(times.size()-1);
    }

    
     /**
     * Get the times ArrayList - not used except for debugging, times is the time at which every sequential part of the spline executes
     * @return the times ArrayList
      * Precondition:  times is not null and has been instantiated
      * Postcondition: times is returned successfully
     */
    private ArrayList<Double> getTimes(){
        if(times == null)
            throw new RuntimeException("Times must not be equal to null in SplinePath.getTimes()");

        return times;
    }

    /**
    * Gets the current arc that is executing based on the current time
    * @param currentTime is the current time since the start of the spline
     * Precondition:  times is not null
     * Postcondition: the appropriate arc is returned
     * @return the arc which the path currently is executing
    */
    private int getArc(double currentTime){
        if(times == null || currentTime < 0)
            throw new RuntimeException("Times must not be equal to null in SplinePath.getArc() and currentTime must be greater than zero");

        for(int i=0;i<times.size();i++)
            if(currentTime < times.get(i))
                return i;
        return -1;
    }
    
    /**
    * Get the linear velocity of the WHOLE robot based off of the current time. It's only != to the velocity specified in the constructor in the ramp-up and ramp-down times.
    * @param currentTime is the current time into the spline
     * Precondition:  currentTime is greater than or equal to zero and arcLengths and times is not null, and the path has been built
     * Postcondition: the appropriate velocity is returned
     * @return the overall robot velocity
    */
    private double getVelocity(double currentTime){
        if(arcLengths == null || times == null || currentTime < 0)
            throw new RuntimeException("currentTime must be greater than or equal to zero and arcLengths and times is not null in SplinePath.getVelocity()");

        int arc = getArc(currentTime);
        if(arc == -1)
            return 0;

        if(arc > 0 && arc < arcLengths.length-1)
            return velocity;
        else if(arc == 0){
            if(currentTime < accelerationTime)
                return velocity*Math.sqrt(currentTime/accelerationTime);
            return velocity;
        }
        else {
            if(currentTime < times.get(times.size()-2)+ additionalPathTwoTime)
                return velocity;
            return velocity-velocity*Math.sqrt((currentTime-times.get(times.size()-2)- additionalPathTwoTime)/accelerationTime);
        }
    }
    
    /**
    * Get the left linear velocity based on the current time, accounting for negative arc lengths, reversed boolean, and different arc radii.
    * @param currentTime is the current time into the spline.
    * @return the left wheel's linear velocity.
     *
     * Precondition:  currentTime is greater than or equal to zero and arcLengths and times is not null, and the path has been built
     * Postcondition: the accurate left velocity is returned
    */
    @Override
    public double getLeftVelocity(double currentTime){
        if(arcLengths == null || times == null)
            throw new RuntimeException("currentTime must be greater than or equal to zero and arcLengths and times is not null in SplinePath.getLeftVelocity()");

        if(moveWay == Direction.FORWARD) {
            if (getArc(currentTime) == -1) {
                this.setCompleted(true);
                return 0;
            }
            double v = getVelocity(currentTime);
            if (arcLengths[getArc(currentTime)] > 0)
                v = v - (v * trackWidth / (2 * radii[getArc(currentTime)]));
            else
                v = v + (v * trackWidth / (2 * radii[getArc(currentTime)]));
            v *= -1;
            return v;
        } else {
            if (getArc(currentTime) == -1) {
                this.setCompleted(true);
                return 0;
            }
            double v = getVelocity(currentTime);
            if (arcLengths[getArc(currentTime)] > 0)
                v = v - (v * trackWidth / (2 * radii[getArc(currentTime)]));
            else
                v = v + (v * trackWidth / (2 * radii[getArc(currentTime)]));
            return v;
        }
    }
    

    /**
    * Get the right linear velocity based on the current time, accounting for negative arc lengths, reversed boolean, and different arc radii.
    * @param currentTime is the current time into the spline.
    * @return the right wheel's linear velocity.
     * Precondition:  currentTime is greater than or equal to zero and arcLengths and times is not null, and the path has been built
     * Postcondition: the accurate right velocity is returned
    */
    @Override
    public double getRightVelocity(double currentTime){
        if(arcLengths == null || times == null)
            throw new RuntimeException("currentTime must be greater than or equal to zero and arcLengths and times is not null in SplinePath.getRightVelocity()");

        if(moveWay == Direction.FORWARD) {
            if (getArc(currentTime) == -1) {
                this.setCompleted(true);
                return 0;
            }
            double v = getVelocity(currentTime);
            if (arcLengths[getArc(currentTime)] > 0)
                return v + (v * trackWidth / (2 * radii[getArc(currentTime)]));
            return v - (v * trackWidth / (2 * radii[getArc(currentTime)]));
        } else {
            if (getArc(currentTime) == -1) {
                this.setCompleted(true);
                return 0;
            }
            double v = getVelocity(currentTime);
            if (arcLengths[getArc(currentTime)] > 0){
                v = v + (v * trackWidth / (2 * radii[getArc(currentTime)]));
                v *= -1;
                return v;
            }
            v = v - (v * trackWidth / (2 * radii[getArc(currentTime)]));
            return -1*v;
        }
    }
    /**
     * Get what type of path this is. Useful for debugging
     * @return The type of path, in this case a spline.
     * Postcondition: the accurate PathType has been returned.
     */
    @Override
    public PathType getType(){
        return PathType.SPLINE;
    }
}

