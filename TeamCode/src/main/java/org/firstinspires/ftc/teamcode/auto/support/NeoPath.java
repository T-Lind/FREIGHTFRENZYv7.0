package org.firstinspires.ftc.teamcode.auto.support;
/**
 * Parent class for paths that solely are composed of time to wheel velocity functions. Essentially acts as a piecewise function over time for both the left and right velocity.
 * @author Tiernan
 * @license MIT License
 */
public abstract class NeoPath {
    private double executeTime = 0;
    private boolean completed = false;
    private boolean built = false;

    public void build(){
        built = true;
    }

    public void setExecuteTime(double d){
        executeTime = d;
    }
    public double getExecuteTime(){
        return executeTime;
    }

    public void setCompleted(boolean b){
        completed = b;
    }
    public boolean getCompleted(){
        return completed;
    }

    public void setBuilt(boolean b){
        built = b;
    }
    public boolean getBuilt(){
        return built;
    }

    public double getLeftVelocity(double t){
        return 0;
    }
    public double getRightVelocity(double t){
        return 0;
    }

    public double convert(double wheelRadius, double v){
        v/=wheelRadius; // convert to angular velocity by radius
        v/=(2*3.14159);
        return v;
    }
}
