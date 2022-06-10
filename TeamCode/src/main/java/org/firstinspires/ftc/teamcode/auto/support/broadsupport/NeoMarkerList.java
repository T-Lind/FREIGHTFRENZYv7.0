package org.firstinspires.ftc.teamcode.auto.support.broadsupport;
import java.util.Arrays;

/**
 * Supporting file to create a static array of markers.
 * @author Tiernan Lindauer
 * For team 7797 Victorian Voltage.
 */
public class NeoMarkerList {
    // Variables to store in this complex list
    private NeoInsertMarker[] markers;
    private double[] times;

    // Maximum amount of markers to allow created
    private int amountOfMarkers;

    /**
     * All of these methods are just meant to nicely create a static array of the NeoNeoInsertMarker implementation.
     * If more than four markers are passed then you must first create them as separate static arrays
     * @param m the marker (listed m, m2, m3, etc.)
     */
    public NeoMarkerList(NeoInsertMarker m, double time1){
        amountOfMarkers = 1;

        markers = new NeoInsertMarker[1];
        markers[0] = m;
        times = new double[amountOfMarkers];
        setTimesMax();
        times[0] = time1;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2){
        amountOfMarkers = 2;

        markers = new NeoInsertMarker[2];
        markers[0] = m;
        markers[1] = m2;
        times = new double[amountOfMarkers];
        setTimesMax();
        times[0] = time1;
        times[1] = time2;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2,
                         NeoInsertMarker m3, double time3){
        amountOfMarkers = 3;

        markers = new NeoInsertMarker[3];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        times = new double[amountOfMarkers];
        setTimesMax();

        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2,
                         NeoInsertMarker m3, double time3, NeoInsertMarker m4, double time4){
        amountOfMarkers = 4;

        markers = new NeoInsertMarker[4];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        times = new double[amountOfMarkers];
        setTimesMax();

        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
    }

    /**
     * Create a list of more than four markers
     * @param markers the list of markers to create
     * @param times the corresponding times to execute those markers
     * @Precondition markers and times must be of the same length
     */
    public NeoMarkerList(NeoInsertMarker[] markers, double[] times){
        if(markers.length != times.length)
            throw new InternalError("The length of markers and times must be the same in NeoMarkerList.NeoMarkerList(...)!");

        amountOfMarkers = markers.length;

        this.markers = markers;
        this.times = times;
    }
    /**
     * Fill the times to the maximum value possible, so that unused InsertMarker slots do not create new threads
     * @Precondition markers is not null
     * @Postcondition every item slot in the array has the maximum value possible so as to not create
     * empty threads, taking up memory.
     */
    private void setTimesMax(){
        assert markers != null : "Cannot run NeoMarkerList.setTimesMax() while markers is null!";
        Arrays.fill(times, Double.MAX_VALUE);
    }

    /**
     * Get the full list of markers
     * @return all of the markers
     * @Precondition markers has been set
     * @Postcondition the list of markers is returned successfully.
     */
    public final NeoInsertMarker[] getMarkers(){
        assert markers != null : "Cannot run NeoMarkerList.getMarkers() while markers is null!";
        return markers;
    }

    /**
     * Get the maximum amount of markers allowed
     * @return the maximum amount of markers
     * @Precondition amountOfMarkers has been set
     * @Postcondition the amount of markers is returned successfully
     */
    public final int getAmountOfMarkers(){
        return amountOfMarkers;
    }

    /**
     * Get the InsertMarker at position i
     * @param i the index to get the InsertMarker in the markers list
     * @return The InsertMarker alias at i
     * @Precondition i must be >= 0
     * @Postcondition the InsertMarker at index i is returned successfully
     */
    public final NeoInsertMarker getNeoInsertMarker(int i){
        assert i >= 0 : "The index for NeoMarkerList.getNeoInsertMarker(int i) must be a valid index!";
        assert markers != null : "The markers array in NeoMarkerList.getNeoInsertMarker(int i) must be not null!";
        return markers[i];
    }

    /**
     * Get the starting time for the marker at position i
     * @param i the index to get the time in the times list
     * @return the starting time for marker i
     * @Precondition times has been set
     * @Postcondition the time at which to execute InsertMarker i has been delivered successfully.
     */
    public final double getTime(int i){
        assert i >= 0 : "The index for NeoMarkerList.getTime(int i) must be a valid index!";
        assert times != null : "The times array in NeoMarkerList.getTime(int i) must be not null!";
        return times[i];
    }

}
