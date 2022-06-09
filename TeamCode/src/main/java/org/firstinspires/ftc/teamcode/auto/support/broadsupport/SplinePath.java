package org.firstinspires.ftc.teamcode.auto.support.broadsupport;
/**
 * Translates a series of circle radii/arc lengths into motor power values for a 2 wheeled robot.
 * created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */

import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.direction;

import java.util.ArrayList;

public class SplinePath extends NeoPath {
    // Variables to characterize the spline
    private double[] radii;
    private double[] arcLengths;
    private double velocity;
    private double trackWidth;
    private double accelerationTime;
    private double additionalpathonetime;
    private double additionalPathTwoTime;
    private direction moveWay;
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
        additionalpathonetime = 0;
        additionalPathTwoTime = 0;

        times = new ArrayList<Double>();
        moveWay = direction.FORWARD;
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
        additionalpathonetime = 0;
        additionalPathTwoTime = 0;

        times = new ArrayList<Double>();

        if(reversed)
            moveWay = direction.REVERSE;
        else
            moveWay = direction.FORWARD;
    }

    /**
     * Compute the various aspects of the spline like ramp-up and ramp-down appending times, the times ArrayList, and ArcLengths list.
     * @Precondition arcLengths and times is not null and has been instantiated
     * @Postcondition times have been successfully computed and this path has been built,
     */
    @Override
    public void build(){
        assert arcLengths != null && times != null : "arcLengths and times must not be null in SplinePath.build()";

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
     * @Precondition times has been instantiated and is not null
     * @Postcondition the appropriate execute time has been returned
     */
    @Override
    public double getExecuteTime(){
        assert times != null : "Times must not be equal to null in SplinePath.getExecuteTime()";

        return times.get(times.size()-1);
    }

    
     /**
     * Get the times ArrayList - not used except for debugging, times is the time at which every sequential part of the spline executes
     * @return the times ArrayList
      * @Precondition times is not null and has been instantiated
      * @Postcondition times is returned successfully
     */
    private ArrayList<Double> getTimes(){
        assert times != null : "Times must not be equal to null in SplinePath.getTimes()";

        return times;
    }

    /**
    * Gets the current arc that is executing based on the current time
    * @param currentTime is the current time since the start of the spline
     * @Precondition times is not null
     * @Postcondition the appropriate arc is returned
    */
    private int getArc(double currentTime){
        assert times != null : "Times must not be equal to null in SplinePath.getArc()";

        for(int i=0;i<times.size();i++)
            if(currentTime < times.get(i))
                return i;
        return -1;
    }
    
    /**
    * Get the linear velocity of the WHOLE robot based off of the current time. It's only != to the velocity specified in the constructor in the ramp-up and ramp-down times.
    * @param currentTime is the current time into the spline
     * @Precondition currentTime is greater than or equal to zero and arcLengths and times is not null, and the path has been built
     * @Postcondition the appropriate velocity is returned
    */
    private double getVelocity(double currentTime){
        assert arcLengths != null && times != null && currentTime >= 0 : "currentTime must be greater than or equal to zero and arcLengths and times is not null in SplinePath.getVelocity()";

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
     * @Precondition currentTime is greater than or equal to zero and arcLengths and times is not null, and the path has been built
     * @Postcondition the accurate left velocity is returned
    */
    @Override
    public double getLeftVelocity(double currentTime){
        assert arcLengths != null && times != null && currentTime >= 0 : "currentTime must be greater than or equal to zero and arcLengths and times is not null in SplinePath.getLeftVelocity()";

        if(moveWay == direction.FORWARD) {
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
     * @Precondition currentTime is greater than or equal to zero and arcLengths and times is not null, and the path has been built
     * @Postcondition the accurate right velocity is returned
    */
    @Override
    public double getRightVelocity(double currentTime){
        assert arcLengths != null && times != null && currentTime >= 0 : "currentTime must be greater than or equal to zero and arcLengths and times is not null in SplinePath.getRightVelocity()";

        if(moveWay == direction.FORWARD) {
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
     * @Postcondition the accurate PathType has been returned.
     */
    public PathType getType(){
        return PathType.SPLINE;
    }
}

