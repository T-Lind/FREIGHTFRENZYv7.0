package org.firstinspires.ftc.teamcode.auto.support;
/**
 * Creates a list of velocities for the wheels on a robot to move at.
 * created by @author Tiernan Lindauer for FTC team 7797.
 * @license  MIT License
 * Last edited 4/26/22
 *
 */

public class Line extends NeoPath{
    private double distance;
    private double maxVelocity;
    private double eT;
    private double errorRate;


    /**
     *
     * @param d is the distance traveled
     * @param v is the maximum velocity to travel at
     */
    public Line(double d, double v){
        errorRate = 1.2;
        distance = d*errorRate;
        maxVelocity = v;

        eT = 0;
    }

    @Override
    public void build(){
        eT = 3.14159265*Math.abs(distance)/(2*maxVelocity);
        setBuilt(true);
    }

    @Override
    public double getExecuteTime(){
        return eT;
    }

    public double getVelocity(double t){
        if(t < eT){
            if(distance > 0)
                return maxVelocity*Math.sin(3.1415925*t/eT);
            return -1*maxVelocity*Math.sin(3.1415925*t/eT);
        }
        else{
            this.setCompleted(true);
            return 0;
        }
    }



    @Override
    public double getLeftVelocity(double t){
        return -getVelocity(t);
    }
    @Override
    public double getRightVelocity(double t){
        return getVelocity(t);
    }
}