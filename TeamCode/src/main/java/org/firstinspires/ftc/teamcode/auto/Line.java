package org.firstinspires.ftc.teamcode.auto;
/**
 * Creates a list of velocities for the wheels on a robot to move at.
 * created by @author Tiernan Lindauer for FTC team 7797.
 * @license  Creative Commons
 * Last edited 4/26/22
 *
 */

class Line extends NeoPath{
    private double distance;
    private double maxVelocity;
    private double eT;
    private double errorRate;


    public Line(double d, double v){
        errorRate = 1.2;
        distance = d*errorRate;
        maxVelocity = v;

        eT = 0;
    }

    @Override
    public void build(){
        eT = 3.14159265*distance/(2*maxVelocity);
    }

    @Override
    public double getExecuteTime(){
        return eT;
    }

    public double getVelocity(double t){
        if(t < eT)
            return maxVelocity*Math.sin(3.1415925*t/eT);
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