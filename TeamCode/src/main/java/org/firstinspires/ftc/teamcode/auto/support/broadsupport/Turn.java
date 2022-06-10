package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.DrivetrainSymmetry;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

/**
 * Program to turn a specified angle.
 * @author Tiernan Lindauer
 * Uses the Line Class and cleverly overrides the left motor velocity.
 */
public class Turn extends Line{
    /**
     * @param angleToTurn is the angle traveled (degrees)
     * @param trackWidth is the distance between the wheels
     * @param maxVelocity is the maximum velocity to travel at
     */
    public Turn(double angleToTurn, double trackWidth, double maxVelocity) {
        super((3.14159*angleToTurn*trackWidth)/360, maxVelocity);
    }

    /**
     *
     * @param angleToTurn is the angle traveled (degrees)
     * @param trackWidth is the distance between the wheels
     * @param maxVelocity is the maximum velocity to travel at
     * @param drivetrainSymmetryType is the drivetrain symmetrical or asymmetrical
     */
    public Turn(double angleToTurn, double trackWidth, double maxVelocity, DrivetrainSymmetry drivetrainSymmetryType) {
        super((3.14159*angleToTurn*trackWidth)/360, maxVelocity, drivetrainSymmetryType);
    }

    /**
     * Get the left wheel velocity
     * @param currentTime the current time
     * @return the left wheel velocity so as to turn the right amount.
     * @Precondition current time is not less than zero
     * @Postcondition the accurate left velocity is returned
     */

    @Override
    public double getLeftVelocity(double currentTime){
        if(currentTime < 0)
            throw new InternalError("currentTime in Turn.getLeftVelocity() must be greater than or equal to zero");
        return -1*super.getLeftVelocity(currentTime);
    }

    /**
     * Get what type of path this is. Useful for debugging
     * @return The type of path, in this case a turn.
     * @Postcondition the correct type is returned
     */
    @Override
    public PathType getType(){
        return PathType.TURN;
    }
}
