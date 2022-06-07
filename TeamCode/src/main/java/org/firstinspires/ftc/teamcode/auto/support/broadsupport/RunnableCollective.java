package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A class for running the markers on different threads at the same time to run each marker.
 * It's a bit of a doozy but I promise these comments will help!
 */
public class RunnableCollective implements Runnable{
    // Create a NeoMarker list - essentially an object containing the InsertMarkers
    // (which are the things we want to do like raise an arm) and the times to execute them.
    private NeoMarkerList markerList;

    // Signal the program when to start running the markers - must be volatile to accomodate the
    // other thread.
    private volatile boolean runMarkers;

    // Signal the program when to stop running the markers
    private boolean stopMarkers;

    /**
     * Instantiate the object with no markerList - used only when creating new threads.
     */
    public RunnableCollective() {
        markerList = null;
        runMarkers = false;
        stopMarkers = false;
    }

    /**
     * Instantiate the object with a markerList to reference, used in the PathSequence family
     * @param markerList
     */
    public RunnableCollective(NeoMarkerList markerList){
        this.markerList = markerList;
        runMarkers = false;
        stopMarkers = false;
    }

    /**
     * Set the run condition
     * @param runMarkers the run condition - either true, do run the markers, or false, wait.
     */
    public void setRunMarkers(boolean runMarkers) {
        this.runMarkers = runMarkers;
    }

    /**
     * Set the stop condition
     * @param stopMarkers the stop condition - either false - don't impact the program - or true,
     *                    halt the program.
     */
    public void setStopMarkers(boolean stopMarkers) {
        this.stopMarkers = stopMarkers;
    }

    /**
     * The broad run method - since the RunnableCollective itself is in a thread it needs to have
     * a run method, which is started whenever the thread is. It instantiates the new threads
     * and starts them as the time becomes right.
     */
    @Override
    public void run() {
        // Create all of the thread objects - at the moment the max is seven (might reduce it)
        Thread t1 = new Thread(new RunnableCollective().new Marker1());
        Thread t2 = new Thread(new RunnableCollective().new Marker2());
        Thread t3 = new Thread(new RunnableCollective().new Marker3());
        Thread t4 = new Thread(new RunnableCollective().new Marker4());
        Thread t5 = new Thread(new RunnableCollective().new Marker5());
        Thread t6 = new Thread(new RunnableCollective().new Marker6());
        Thread t7 = new Thread(new RunnableCollective().new Marker7());

        // Declare the ElapsedTime object here to reduce lag after runMarkers is true
        ElapsedTime markerTime = new ElapsedTime();

        // Wait until the signal has been given to run the markers
        while(!runMarkers);

        // Reset the time the markers run off of (should be whenever start button is pressed)
        markerTime.reset();

        // Look and start the threads if the time is right and they have not been started before
        while(!stopMarkers){
            if(markerList.getTime(0) > markerTime.milliseconds()/1000 && !t1.isAlive())
                t1.start();
            if(markerList.getTime(1) > markerTime.milliseconds()/1000 && !t2.isAlive())
                t2.start();
            if(markerList.getTime(2) > markerTime.milliseconds()/1000 && !t3.isAlive())
                t3.start();
            if(markerList.getTime(3) > markerTime.milliseconds()/1000 && !t4.isAlive())
                t4.start();
            if(markerList.getTime(4) > markerTime.milliseconds()/1000 && !t5.isAlive())
                t5.start();
            if(markerList.getTime(5) > markerTime.milliseconds()/1000 && !t6.isAlive())
                t6.start();
            if(markerList.getTime(6) > markerTime.milliseconds()/1000 && !t7.isAlive())
                t7.start();
            }
        }


    /**
     * First thread object - correlates to index 0 of the markerList.
     */
    private class Marker1 implements Runnable{
        @Override
        public void run() {
            markerList.getNeoInsertMarker(0).execute();
        }
    }

    /**
     * Second thread object - correlates to index 1 of the markerList.
     */
    private class Marker2 implements Runnable{
        @Override
        public void run() {
            markerList.getNeoInsertMarker(1).execute();
        }
    }

    /**
     * Third thread object - correlates to index 2 of the markerList.
     */
    private class Marker3 implements Runnable{
        @Override
        public void run() {
            markerList.getNeoInsertMarker(2).execute();
        }
    }

    /**
     * Fourth thread object - correlates to index 3 of the markerList.
     */
    private class Marker4 implements Runnable{
        @Override
        public void run() {
            markerList.getNeoInsertMarker(3).execute();
        }
    }

    /**
     * Fifth thread object - correlates to index 4 of the markerList.
     */
    private class Marker5 implements Runnable{
        @Override
        public void run() {
            markerList.getNeoInsertMarker(4).execute();
        }
    }

    /**
     * Sixth thread object - correlates to index 5 of the markerList.
     */
    private class Marker6 implements Runnable{
        @Override
        public void run() {
            markerList.getNeoInsertMarker(5).execute();
        }
    }

    /**
     * Seventh thread object - correlates to index 6 of the markerList.
     */
    private class Marker7 implements Runnable{
        @Override
        public void run() {
            markerList.getNeoInsertMarker(6).execute();
        }
    }
}
