package org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport;

import org.firstinspires.ftc.teamcode.auto.coyotesupport.enumerations.PathType;

public class MecLine extends MecPath{
    /**
     * What angle the robot is moving in a line at - unit circle convention, in degrees
     */
    private double angle;

    /**
     * Distance to travel at in the line
     */
    private double distance;

    /**
     * Maximum velocity to travel at
     */
    private double maxVelocity;

    private double posDiagonalVelocity;

    private double negDiagonalVelocity;

    public MecLine(double angle, double distance, double maxVelocity){
        this.angle = angle;
        this.distance = distance;
        this.maxVelocity = maxVelocity;
    }

    /**
     * Method to get the type of path this path is
     * @return the path type MECLINE
     */
    @Override
    public PathType getType() {
        return PathType.MECLINE;
    }

    /**
     * Build the MecLine type path
     */
    @Override
    public void build() {
        posDiagonalVelocity = Math.sin(Math.toRadians(angle) - 1.0 / 4 * Math.PI);
        negDiagonalVelocity = Math.sin(Math.toRadians(angle) + 1.0 / 4 * Math.PI);
        setExecuteTime(3.14159265*Math.abs(distance)/(2*maxVelocity));
        setBuilt(true);
    }

    @Override
    public double getLeftFrontVelocity(double time) {
        return getLinearVelocity(time)*negDiagonalVelocity;
    }

    @Override
    public double getLeftBackVelocity(double time) {
        return getLinearVelocity(time)*posDiagonalVelocity;
    }

    @Override
    public double getRightFrontVelocity(double time) {
        return getLeftBackVelocity(time);
    }

    @Override
    public double getRightBackVelocity(double time) {
        return getLeftFrontVelocity(time);
    }

    private double getLinearVelocity(double t) {
        if(t < getExecuteTime()){
            if(distance > 0)
                return maxVelocity*Math.sin(3.1415925*t/ getExecuteTime());
            return -1*maxVelocity*Math.sin(3.1415925*t/ getExecuteTime());
        }
        else{
            this.setCompleted(true);
            return 0;
        }
    }
}
