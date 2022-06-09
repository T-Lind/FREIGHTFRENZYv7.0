package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * A class for running the markers on different threads at the same time to run each marker.
 */
public class RunnableCollective implements Runnable {
    // Create a NeoMarker list - essentially an object containing the InsertMarkers
    // (which are the things we want to do like raise an arm) and the times to execute them.
    private final NeoMarkerList markerList;

    // Signal the program when to stop running the markers
    private volatile static boolean stopMarkers;

    // Threads
    private static Thread mainThread;
    private static Thread t1;
    private static Thread t2;
    private static Thread t3;
    private static Thread t4;
    private static Thread t5;
    private static Thread t6;
    private static Thread t7;

    /**
     * Instantiate the object with a markerList to reference, used in the PathSequence family
     *
     * @param markerList is the collection of markers and the time they should execute at
     * @Precondition markerList is not null
     * @Postcondition markerList is unchanged and stopMarkers has been set to false.
     */
    public RunnableCollective(NeoMarkerList markerList) {
        assert markerList != null : "markerList cannot be null in this instance!";
        this.markerList = markerList;
        stopMarkers = false;
    }

    /**
     * Start the thread that runs all of the markers
     *
     * @Postcondition the markers have started
     */
    public final void activateMarkers() {
        mainThread = new Thread(new RunnableCollective(markerList));
        mainThread.start();
    }


    /**
     * Set the stop condition and interrupts threads
     *
     * @Postcondition the markers have stopped running
     */
    public final void setStopMarkers() {
        // Interrupt any child thread currently being used
        RunnableCollective.stopMarkers = true;
        if (t1.isAlive())
            t1.interrupt();
        if (t2.isAlive())
            t2.interrupt();
        if (t3.isAlive())
            t2.interrupt();
        if (t4.isAlive())
            t2.interrupt();
        if (t5.isAlive())
            t2.interrupt();
        if (t6.isAlive())
            t2.interrupt();
        if (t7.isAlive())
            t2.interrupt();

        // Interrupt the parent thread
        mainThread.interrupt();
    }

    /**
     * The broad run method - since the RunnableCollective itself is in a thread it needs to have
     * a run method, which is started whenever the thread is. It instantiates the new threads
     * and starts them as the time becomes right.
     *
     * @Postcondition the markers required have been created
     */
    @Override
    public final void run() {

        // Create all of the thread objects - at the moment the max is seven (might reduce it)
        t1 = new Thread(new RunnableCollective(markerList).new MarkerThread(0));
        t2 = new Thread(new RunnableCollective(markerList).new MarkerThread(1));
        t3 = new Thread(new RunnableCollective(markerList).new MarkerThread(2));
        t4 = new Thread(new RunnableCollective(markerList).new MarkerThread(3));
        t5 = new Thread(new RunnableCollective(markerList).new MarkerThread(4));
        t6 = new Thread(new RunnableCollective(markerList).new MarkerThread(5));
        t7 = new Thread(new RunnableCollective(markerList).new MarkerThread(6));

        // Declare the ElapsedTime object here to reduce lag after runMarkers is true
        ElapsedTime markerTime = new ElapsedTime();

        // Reset the time the markers run off of (should be whenever start button is pressed)
        markerTime.reset();

        // Look and start the threads if the time is right and they have not been started before
        while (!stopMarkers) {
            if (markerList.getTime(0) < markerTime.milliseconds() / 1000 && !t1.isAlive())
                t1.start();
            if (markerList.getTime(1) < markerTime.milliseconds() / 1000 && !t2.isAlive())
                t2.start();
            if (markerList.getTime(2) < markerTime.milliseconds() / 1000 && !t3.isAlive())
                t3.start();
            if (markerList.getTime(3) < markerTime.milliseconds() / 1000 && !t4.isAlive())
                t4.start();
            if (markerList.getTime(4) < markerTime.milliseconds() / 1000 && !t5.isAlive())
                t5.start();
            if (markerList.getTime(5) < markerTime.milliseconds() / 1000 && !t6.isAlive())
                t6.start();
            if (markerList.getTime(6) < markerTime.milliseconds() / 1000 && !t7.isAlive())
                t7.start();
        }
    }


    /**
     * First thread object - correlates to index 0 of the markerList.
     */
    private final class MarkerThread implements Runnable {
        private int index;

        public MarkerThread(int index){
            assert index < markerList.getMarkers().length : "Error: tried to get an index for a marker that does not exist!";
            this.index = index;
        }

        /**
         * Method to execute the neoInsertMarker
         *
         * @Precondition markerList is not null and the index provided exists in the array of markers
         * @Postcondition the marker at this spot has started
         */
        @Override
        public void run() {
            assert markerList != null : "Error occurred: markerList was null and yet a marker still tried to execute!";
            markerList.getNeoInsertMarker(index).execute();
        }
    }
}
