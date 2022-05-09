package org.firstinspires.ftc.teamcode.auto.support;
/**
 * Parent class for paths that solely are composed of time to wheel velocity functions. Essentially acts as a piecewise function over time for both the left and right velocity.
 * @license MIT License
 * Last edited 5/5
 */
public abstract class NeoPath {
    private double executeTime = 0;
    private boolean completed = false;
    private boolean built = false;

    /**
     * Meant to compute the trajectory and essentially turn it into a piecewise function.
     * This method is supposed to be overriden.
     */
    public void build(){
        built = true;
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
        built = b;
    }
    /**
    * Get the build status of this NeoPath.
    * @return the built variable (build status).
    */
    public boolean getBuilt(){
        return built;
    }

    /**
    * Get the left velocity - meant to be overriden, defaults to zero speed.
    * @param t is the current time into this specific NeoPath.
    */
    public double getLeftVelocity(double t){
        return 0;
    }
    /**
    * Get the right velocity - meant to be overriden, defaults to zero speed.
    * @param t is the current time into this specific NeoPath.
    */
    public double getRightVelocity(double t){
        return 0;
    }

    /**
     *
     * @param wheelRadius is the radius, NOT DIAMETER, of the wheel
     * @param v is the linear velocity in m/s
     * @return the angular velocity in rad/s
     */
    public double convert(double wheelRadius, double v){
        v/=wheelRadius; // convert to angular velocity by radius
        v/=(2*3.14159);
        return v;
    }
}
