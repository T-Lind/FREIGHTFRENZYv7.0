package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.DrivetrainSymmetry;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Creates a list of velocities for the wheels on a robot to move at.
 * created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */

public class Line extends Path {
    // Variables to model the type of line this path should follow

    /**
     * Distance this path should go
     */
    private final double distance;

    /**
     * Maximum velocity the robot should reach
     */
    private final double maxVelocity;

    /**
     * Time it takes to complete this path
     */
    private double executeTime;

    /**
     * What kind of symmetry does the drivetrain have - important because motors may have to run in reverse to make a line
     */
    private final DrivetrainSymmetry drivetrainSymmetryType;

    /**
     * Constructor for this Line object. Here it make an assumption since it has not been given
     * enough data - sets the drivtrain type to asymmetrical
     * @param distance is the distance traveled
     * @param maxVelocity is the maximum velocity to travel at
     */
    public Line(double distance, double maxVelocity){
        if(maxVelocity == 0 || distance == 0)
            throw new RuntimeException("The velocity or distance must not be equal to zero in Line.Line(...)!");
        this.distance = distance;
        this.maxVelocity = maxVelocity;

        executeTime = 0;

        drivetrainSymmetryType = DrivetrainSymmetry.ASYMMETRICAL;
    }
    /**
     *Constructor for this Line object. Sets the type of drivetrain to
     * @param distance is the distance traveled
     * @param maxVelocity is the maximum velocity to travel at
     * @param drivetrainSymmetryType is if the drivetrain is not mirrored across the axis (like in a differential swerve)
     */
    public Line(double distance, double maxVelocity, DrivetrainSymmetry drivetrainSymmetryType){
        if(maxVelocity == 0 || distance == 0)
            throw new RuntimeException("The velocity or distance must not be equal to zero in Line.Line(...)!");
        this.distance = distance;
        this.maxVelocity = maxVelocity;
        executeTime = 0;

        this.drivetrainSymmetryType = drivetrainSymmetryType;
    }
    /**
     * Build the line trajectory.
     * Postcondition: this path has been built successfully
     */
    @Override
    public final void build(){
        executeTime = 3.14159265*Math.abs(distance)/(2*maxVelocity);
        setBuilt(true);
    }

    /**
     * Get the time it takes to run the path
     * @return the execution time
     */
    @Override
    public final double getExecuteTime(){
        return executeTime;
    }

    /**
     * Get the linear velocity of the entire bot
     * @param t the current time
     * @return the linear velocity of the bot
     */
    private double getVelocity(double t){
        if(t < executeTime){
            if(distance > 0)
                return maxVelocity*Math.sin(3.1415925*t/ executeTime);
            return -1*maxVelocity*Math.sin(3.1415925*t/ executeTime);
        }
        else{
            this.setCompleted(true);
            return 0;
        }
    }


    /**
     * Get the left linear velocity
     * @param currentTime the current time
     * @return left linear velocity
     */
    @Override
    public double getLeftVelocity(double currentTime){
        if(drivetrainSymmetryType == DrivetrainSymmetry.ASYMMETRICAL)
            return -getVelocity(currentTime);
        return getVelocity(currentTime);
    }
    /**
     * Get the right linear velocity
     * @param currentTime the current time
     * @return right linear velocity
     */
    @Override
    public double getRightVelocity(double currentTime){
        return getVelocity(currentTime);
    }

    /**
     * Get what type of path this is. Useful for debugging
     * @return The type of path, in this case a line.
     */
    @Override
    public PathType getType(){
        return PathType.LINE;
    }
}