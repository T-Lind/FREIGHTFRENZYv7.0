package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

public class MecLine extends Path{
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
    public void build(){

    }

    public double getLeftFrontVelocity(double time) {
        return 0; // TODO: Work out the math to find each of these linear velocities
    }

    public double getLeftBackVelocity(double time) {
        return 0; // TODO: Work out the math to find each of these linear velocities
    }

    public double getRightFrontVelocity(double time) {
        return 0; // TODO: Work out the math to find each of these linear velocities
    }

    public double getRightBackVelocity(double time) {
        return 0; // TODO: Work out the math to find each of these linear velocities
    }
}
