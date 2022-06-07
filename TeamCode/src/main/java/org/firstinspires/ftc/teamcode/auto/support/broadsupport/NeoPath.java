package org.firstinspires.ftc.teamcode.auto.support.broadsupport;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.BuildStatus;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Parent class for paths that solely are composed of time to wheel velocity functions. Essentially acts as a piecewise function over time for both the left and right velocity.
 */
public abstract class NeoPath {
    private double executeTime = 0;
    private boolean completed = false;
    private BuildStatus construction = BuildStatus.UNBUILT;

    /**
     * Method returning the type of
     * @return
     */
    abstract public PathType getType();

    /**
     * Meant to compute the trajectory and essentially turn it into a piecewise function.
     * This method is supposed to be overriden.
     */
    public void build(){
        construction = BuildStatus.BUILT;
    }

    /**
    * Set the execute time - time it takes to follow this specific NeoPath.
    * @param d is the new execute time.
    */
    public void setExecuteTime(double d){
        executeTime = d;
    }
    /**
    * Get the current execute time - time it takes to follow this specific NeoPath.
    * @return the execute time.
    */
    public double getExecuteTime(){
        return executeTime;
    }

    /**
    * Set the completed status - not overriden.
    * @param b is the new status of the path (always going to be true).
    */
    public void setCompleted(boolean b){
        completed = b;
    }
    
    /**
    * Get the completed status - not overriden.
    * @return the completed status.
    */
    public boolean getCompleted(){
        return completed;
    }

    /**
    * Set the built boolean.
    * @param b is the build status.
    */
    public void setBuilt(boolean b){
        if(b)
            construction = BuildStatus.BUILT;
        else
            construction = BuildStatus.UNBUILT;
    }
    /**
    * Get the build status of this NeoPath.
    * @return the built variable (build status).
    */
    public boolean getBuilt(){
        if(construction == BuildStatus.BUILT)
            return true;
        return false;
    }

    /**
    * Get the left velocity - meant to be overridden, defaults to zero speed.
    * @param t is the current time into this specific NeoPath.
    */
    public double getLeftVelocity(double t){
        return 0;
    }
    /**
    * Get the right velocity - meant to be overridden, defaults to zero speed.
    * @param t is the current time into this specific NeoPath.
    */
    public double getRightVelocity(double t){
        return 0;
    }

    /**
     * Get the left angle - meant to be overridden, defaults to zero.
     * @param t is the current time into this specific NeoPath.
     */
    public double getLeftAngle(double t){
        return 0;
    }

    /**
     * Get the right angle - meant to be overridden, defaults to zero.
     * @param t is the current time into this specific NeoPath.
     */
    public double getRightAngle(double t){
        return 0;
    }

    /**
     *
     * @param wheelRadius is the radius, NOT DIAMETER, of the wheel
     * @param v is the linear velocity in m/s
     * @return the angular velocity in rad/s
     */
    public static double convert(double wheelRadius, double v){
        v/=wheelRadius; // convert to angular velocity by radius
        v/=(2*3.14159);
        return v;
    }
}
