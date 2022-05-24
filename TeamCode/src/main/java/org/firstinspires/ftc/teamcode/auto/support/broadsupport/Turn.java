package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

/**
 * Program to turn a specified angle.
 * @author Tiernan Lindauer
 * Uses the Line Class and cleverly overrides the left motor velocity.
 */
public class Turn extends Line{
    /**
     * @param a is the angle traveled (degrees)
     * @param v is the maximum velocity to travel at
     */
    public Turn(double a, double trackWidth, double v) {
        super((3.14159*a*trackWidth)/360, v);
    }

    public Turn(double a, double trackWidth, double v, boolean notSymmetrical) {
        super((3.14159*a*trackWidth)/360, v, notSymmetrical);
    }

    /**
     * Get the left wheel velocity
     * @param t the current time
     * @return the left wheel velocity so as to turn the right amount.
     */

    @Override
    public double getLeftVelocity(double t){
        return -1*super.getLeftVelocity(t);
    }
}
