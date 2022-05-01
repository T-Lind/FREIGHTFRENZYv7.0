package org.firstinspires.ftc.teamcode.auto;
/**
 * Parent class for paths that solely are composed of time to wheel velocity functions
 * created by @author Tiernan Lindauer for FTC team 7797.
 * @license  Creative Commons
 * Last edited 4/18/22
 *
 */
public class NeoPath {
    private double executeTime = 0;
    private boolean completed = false;

    public void build(){
        return;
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

    public double getLeftVelocity(double t){
        return 0;
    }
    public double getRightVelocity(double t){
        return 0;
    }

    public double convert(double v){
        v/=0.048; // convert to angular velocity by radius
        v/=(2*3.14159);
        return v;
    }
}
