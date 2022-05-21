package org.firstinspires.ftc.teamcode.auto.support.diffysupport;

import org.firstinspires.ftc.teamcode.auto.support.NeoPath;


public class DiffyLine extends NeoPath {
    private double distance;
    private double maxVelocity;
    private double eT;
    private double errorRate;


    /**
     *
     * @param d is the distance traveled
     * @param v is the maximum velocity to travel at
     */
    public DiffyLine(double d, double v){
        distance = d;
        maxVelocity = v;

        eT = 0;
    }

    /**
     * Build the line trajectory.
     */
    @Override
    public void build(){
        eT = 3.14159265*Math.abs(distance)/(2*maxVelocity);
        setBuilt(true);
    }

    /**
     * Get the time it takes to run the path
     * @return the execution time
     */
    @Override
    public double getExecuteTime(){
        return eT;
    }

    /**
     * Get the linear velocity of the entire bot
     * @param t the current time
     * @return the linear velocity of the bot
     */
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


    /**
     * Get the left linear velocity
     * @param t the current time
     * @return left linear velocity
     */
    @Override
    public double getLeftVelocity(double t){
        return -getVelocity(t);
    }
    /**
     * Get the right linear velocity
     * @param t the current time
     * @return right linear velocity
     */
    @Override
    public double getRightVelocity(double t){
        return getVelocity(t);
    }
}