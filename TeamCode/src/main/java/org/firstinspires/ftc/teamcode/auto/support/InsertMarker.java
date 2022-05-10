package org.firstinspires.ftc.teamcode.auto.support;

/**
 * An interface to execute different parts of code at different times in the trajectory.
 * Useful for running arms, motors, etc. while in movement.
 * @author Tiernan Lindauer
 */
public interface InsertMarker {
    /**
     * execute is what occurs based on the current time
     * @param t the amount of time in seconds since the path sequence has been started.
     */
    void execute(double t);
}
