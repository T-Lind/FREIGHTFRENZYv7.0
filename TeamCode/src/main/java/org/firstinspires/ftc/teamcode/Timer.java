package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Basic class to make a simple
 */
public class Timer {
    private double timeToLoop;
    private ElapsedTime elapsedTime;

    // Acceptable margin of error for time measurement in seconds for the timer
    private static final double timerMarginOfError = 0.5;

    /**
     * Establish a timer object. Not used for the static method sleep.
     * @param timeToLoop the amount of time to be in a loop.
     */
    public Timer(double timeToLoop){
        this.timeToLoop = timeToLoop;
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
    }

    /**
     * Check to see if the current time for this Timer instance is past the time to loop
     * @return whether or not to continue looping
     */
    public boolean inTimeRange(){
        return elapsedTime.milliseconds() / 1000 < timeToLoop;
    }

    /**
     * Gets the current time of this Timer instance
     * @return the current time in seconds of this Timer instance
     */
    public double getTime(){
        return elapsedTime.milliseconds()/1000;
    }

    /**
     * Hang up the thread based on a time to sleep
     * @param timeToSleep the time the thread should sleep for
     */
    public static void sleep(double timeToSleep){
        ElapsedTime currentTime = new ElapsedTime();
        while(currentTime.milliseconds()/1000+timerMarginOfError < timeToSleep);
    }

}
