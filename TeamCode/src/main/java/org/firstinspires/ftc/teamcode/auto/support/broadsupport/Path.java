package org.firstinspires.ftc.teamcode.auto.support.broadsupport;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.BuildStatus;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Parent class for paths that solely are composed of time to wheel velocity functions.
 * Essentially acts as a piecewise function over time for both the left and right velocity.
 */
public abstract class Path {
    // Variables common to each path.
    private double executeTime = 0;
    private boolean completed = false;
    private BuildStatus construction = BuildStatus.UNBUILT;

    /**
     * Method returning the type of path this specific path is
     * @return the correct type of path.
     */
    abstract public PathType getType();

    /**
     * Meant to compute the trajectory and essentially turn it into a piecewise function.
     * This method is supposed to be overriden.
     * @Precondition the construction variable has been instantiated
     * @Postcondition the path is set to a built status
     */
    public void build(){
        construction = BuildStatus.BUILT;
    }

    /**
    * Set the execute time - time it takes to follow this specific NeoPath.
    * @param executeTime is the new execute time.
     * @Precondition execute time is greater than zero
     * @Postcondition the execute time has been set
    */
    public final void setExecuteTime(double executeTime){
        assert executeTime > 0 : "The execute time in NeoPath.setExecuteTime(...) must be greater than zero!";
        this.executeTime = executeTime;
    }
    /**
    * Get the current execute time - time it takes to follow this specific NeoPath.
    * @return the execute time.
    */
    public double getExecuteTime(){
        assert executeTime > 0 : "The execute time in NeoPath.getExecuteTime(...) must be greater than zero!";
        return executeTime;
    }

    /**
    * Set the completed status - not overriden.
    * @param completionStatus is the new status of the path (should always be assigned as true).
     * @Precondition the completion status parameter accurately reflects the state of the robot
     * @Postcondition the completed variable is set
    */
    public void setCompleted(boolean completionStatus){
        completed = completionStatus;
    }
    
    /**
    * Get the completed status - not overriden.
    * @return the completed status.
     * @Precondition the completed status has been accurately updated
     * @Postcondition the status of completion has been returned
    */
    public final boolean getCompleted(){
        return completed;
    }

    /**
    * Set the built boolean.
    * @param isBuilt is the build status.
     * @Precondition the isBuilt parameter accurately reflects the status of the path
     * @Postcondition the construction variable is accurately set
    */
    public final void setBuilt(boolean isBuilt){
        if(isBuilt)
            construction = BuildStatus.BUILT;
        else
            construction = BuildStatus.UNBUILT;
    }
    /**
    * Get the build status of this NeoPath.
    * @return the built variable (build status).
     * @Precondition the build status has been accurately updated
     * @Postcondition the accurate build status has been returned
    */
    public final boolean getBuilt(){
        return construction == BuildStatus.BUILT;
    }

    /**
    * Get the left velocity - meant to be overridden, defaults to zero speed.
    * @param currentTime is the current time into this specific NeoPath.
     * @Precondition
    */
    public double getLeftVelocity(double currentTime){
        assert currentTime >= 0 : "The time to get the left velocity for in NeoPath.getLeftVelocity(...) must not be less than zero!";
        return 0;
    }
    /**
    * Get the right velocity - meant to be overridden, defaults to zero speed.
    * @param currentTime is the current time into this specific NeoPath.
    */
    public double getRightVelocity(double currentTime){
        assert currentTime >= 0 : "The time to get the right velocity for in NeoPath.getRightVelocity(...) must not be less than zero!";
        return 0;
    }

    /**
     * Get the left angle - meant to be overridden, defaults to zero.
     * @param currentTime is the current time into this specific NeoPath.
     */
    public double getLeftAngle(double currentTime){
        assert currentTime >= 0 : "The time to get the left velocity for in NeoPath.getLeftAngle(...) must not be less than zero!";
        return 0;
    }

    /**
     * Get the right angle - meant to be overridden, defaults to zero.
     * @param currentTime is the current time into this specific NeoPath.
     */
    public double getRightAngle(double currentTime){
        assert currentTime >= 0 : "The time to get the right velocity for in NeoPath.getRightAngle(...) must not be less than zero!";
        return 0;
    }

    /**
     *
     * @param wheelRadius is the radius, NOT DIAMETER, of the wheel
     * @param velocity is the linear velocity in m/s
     * @return the angular velocity in rad/s
     */
    public static double convert(double wheelRadius, double velocity){
        assert wheelRadius > 0 && velocity > 0 : "Wheel radius and velocity must be greater than zero in NeoPath.convert(...)";
        velocity/=wheelRadius; // convert to angular velocity by radius
        velocity/=(2*3.14159);
        return velocity;
    }
}
