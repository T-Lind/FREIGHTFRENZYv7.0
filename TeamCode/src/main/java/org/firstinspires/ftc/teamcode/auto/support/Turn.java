package org.firstinspires.ftc.teamcode.auto.support;

public class Turn extends Line{
    /**
     * @param a is the angle traveled (degrees)
     * @param v is the maximum velocity to travel at
     */
    public Turn(double a, double trackWidth, double v) {
        super((3.14159*a*trackWidth)/360, v);
    }

    @Override
    public double getLeftVelocity(double t){
        return -1*super.getLeftVelocity(t);
    }
}
