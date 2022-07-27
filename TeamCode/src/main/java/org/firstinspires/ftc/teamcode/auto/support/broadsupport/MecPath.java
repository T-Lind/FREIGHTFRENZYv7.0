package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

abstract public class MecPath extends Path{
    abstract public double getLeftFrontVelocity(double time);

    abstract public double getLeftBackVelocity(double time);

    abstract public double getRightFrontVelocity(double time);

    abstract public double getRightBackVelocity(double time);
}
