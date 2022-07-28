package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

abstract public class MecPath extends Path{
    /**
     * Method to get the left front velocity given a certain time into this path
     * @param time the time into this path
     * @return the velocity in rad/s for the motor
     */
    abstract public double getLeftFrontVelocity(double time);

    /**
     * Method to get the left back velocity given a certain time into this path
     * @param time the time into this path
     * @return the velocity in rad/s for the motor
     */
    abstract public double getLeftBackVelocity(double time);

    /**
     * Method to get the right front velocity given a certain time into this path
     * @param time the time into this path
     * @return the velocity in rad/s for the motor
     */
    abstract public double getRightFrontVelocity(double time);

    /**
     * Method to get the right back velocity given a certain time into this path
     * @param time the time into this path
     * @return the velocity in rad/s for the motor
     */
    abstract public double getRightBackVelocity(double time);
}
