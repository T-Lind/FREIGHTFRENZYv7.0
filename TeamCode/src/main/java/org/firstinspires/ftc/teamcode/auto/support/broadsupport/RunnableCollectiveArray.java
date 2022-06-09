package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Thread object - correlates to index 0 of the markerList.
 */
final class ChildThread implements Runnable{
    private NeoMarkerList markerList;

    public ChildThread(NeoMarkerList markerList){
        this.markerList = markerList;
    }

    /**
     * Method to execute the neoInsertMarker
     *@Precondition markerList is not null
     *@Postcondition the marker at this spot has started
     */
    @Override
    public void run() {
        markerList.getNeoInsertMarker(0).execute();
    }
}

/**
 * A class for running the markers on different threads at the same time to run each marker.
 * Unlike the typical RunnableCollective, this can create as many threads as you want!
 */
public class RunnableCollectiveArray implements Runnable{
    // Create a NeoMarker list - essentially an object containing the InsertMarkers
    // (which are the things we want to do like raise an arm) and the times to execute them.
    private final NeoMarkerList markerList;

    // Signal the program when to stop running the markers
    private boolean stopMarkers;

    private Thread mainThread;
    private Thread[] childThreadList;

    /**
     * Instantiate the object with a markerList to reference, used in the PathSequence family
     * @param markerList is the collection of markers and the time they should execute at
     */
    public RunnableCollectiveArray(NeoMarkerList markerList){
        assert markerList != null : "markerList cannot be null in this instance!";
        this.markerList = markerList;
        childThreadList = new Thread[markerList.getMarkers().length];
        stopMarkers = false;
    }

    /**
     * Start the thread that runs all of the markers
     * @Precondition markerList is not null and has been instantiated
     * @Postcondition the main thread has started
     */
    public final void activateMarkers(){
        assert markerList != null : "markerList in RunnableCollectiveArray.activateMarkers() must not be null!";
        mainThread = new Thread(new RunnableCollective(markerList));
        mainThread.start();
    }


    /**
     * Set the stop condition and interrupts threads
     * @Precondition childThreadList has been instantiated and is not null
     * @Postcondition the markers have stopped running
     */
    public final void setStopMarkers() {
        assert childThreadList != null : "childThreadList in RunnableCollectiveArray.setStopMarkers() must not be null!";

        // Interrupt any child thread currently being used
        stopMarkers = true;
        for(int i=0;i<childThreadList.length;i++)
            if(childThreadList[i] != null && childThreadList[i].isAlive())
                childThreadList[i].interrupt();

        // Interrupt the parent thread
        mainThread.interrupt();
    }

    /**
     * The broad run method - since the RunnableCollective itself is in a thread it needs to have
     * a run method, which is started whenever the thread is. It instantiates the new threads
     * and starts them as the time becomes right.
     * @Precondition childThreadList and markerList has been instantiated and are not null
     * @Postcondition each child thread has been started and executed according to the interface
     * realization with lambda in the runner program
     */
    @Override
    public final void run(){
        assert markerList != null : "markerList in RunnableCollectiveArray.activateMarkers() must not be null!";
        assert childThreadList != null : "childThreadList in RunnableCollectiveArray.setStopMarkers() must not be null!";


        // Declare the ElapsedTime object here to reduce lag after runMarkers is true
        ElapsedTime markerTime = new ElapsedTime();

        // Reset the time the markers run off of (should be whenever start button is pressed)
        markerTime.reset();

        // Create each new thread in the array
        for(int i=0;i< childThreadList.length;i++)
            childThreadList[i] = new Thread(new ChildThread(markerList));

        // Look and start the threads if the time is right and they have not been started before
        while(!stopMarkers)
            for(int i=0;i<childThreadList.length;i++)
                if(markerList.getTime(i) < markerTime.milliseconds()/1000 && !childThreadList[i].isAlive())
                    childThreadList[i].start();
    }
}
