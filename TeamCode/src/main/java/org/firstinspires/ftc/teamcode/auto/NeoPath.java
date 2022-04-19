package org.firstinspires.ftc.teamcode.auto;
/**
 * Parent class for paths that solely are composed of varying
 * motor power values.
 * created by @author Tiernan Lindauer for FTC team 7797.
 * @license  Creative Commons
 * Last edited 4/18/22
 *
 */
public class NeoPath {
    private double executeTime = 0;

    public void build(){
        return;
    }

    public void setExecuteTime(double d){
        executeTime = d;
    }
    public double getExecuteTime(){
        return executeTime;
    }

    public double getLeftVelocity(double t){
        return 0;
    }
    public double getRightVelocity(double t){
        return 0;
    }
}
