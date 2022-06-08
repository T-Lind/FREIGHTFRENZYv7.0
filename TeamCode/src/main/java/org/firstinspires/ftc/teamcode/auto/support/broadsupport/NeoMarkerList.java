package org.firstinspires.ftc.teamcode.auto.support.broadsupport;
import java.util.Arrays;

/**
 * Supporting file to create a static array of markers.
 * @author Tiernan Lindauer
 * For team 7797 Victorian Voltage.
 */
public class NeoMarkerList {
    private NeoInsertMarker[] markers;
    private double times[];
    private final int MAX_AMOUNT_MARKERS = 7;

    /**
     * All of these methods are just meant to nicely create a static array of the NeoNeoInsertMarker implementation.
     * @param m the marker (listed m, m2, m3, etc.)
     */
    public NeoMarkerList(NeoInsertMarker m, double time1){
        markers = new NeoInsertMarker[1];
        markers[0] = m;
        times = new double[MAX_AMOUNT_MARKERS];
        setTimesMax();
        times[0] = time1;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2){
        markers = new NeoInsertMarker[2];
        markers[0] = m;
        markers[1] = m2;
        times = new double[MAX_AMOUNT_MARKERS];
        setTimesMax();
        times[0] = time1;
        times[1] = time2;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2,
                         NeoInsertMarker m3, double time3){
        markers = new NeoInsertMarker[3];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        times = new double[MAX_AMOUNT_MARKERS];
        setTimesMax();

        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2,
                         NeoInsertMarker m3, double time3, NeoInsertMarker m4, double time4){
        markers = new NeoInsertMarker[4];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        times = new double[MAX_AMOUNT_MARKERS];
        setTimesMax();

        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2,
                         NeoInsertMarker m3, double time3, NeoInsertMarker m4, double time4,
                         NeoInsertMarker m5, double time5){
        markers = new NeoInsertMarker[5];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        times = new double[MAX_AMOUNT_MARKERS];
        setTimesMax();
        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
        times[4] = time5;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2,
                         NeoInsertMarker m3, double time3, NeoInsertMarker m4, double time4,
                         NeoInsertMarker m5, double time5, NeoInsertMarker m6, double time6){
        markers = new NeoInsertMarker[6];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        markers[5] = m6;
        times = new double[MAX_AMOUNT_MARKERS];
        setTimesMax();

        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
        times[4] = time5;
        times[5] = time6;
    }
    public NeoMarkerList(NeoInsertMarker m, double time1, NeoInsertMarker m2, double time2,
                         NeoInsertMarker m3, double time3, NeoInsertMarker m4, double time4,
                         NeoInsertMarker m5, double time5, NeoInsertMarker m6, double time6,
                         NeoInsertMarker m7, double time7){
        markers = new NeoInsertMarker[7];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        markers[5] = m6;
        markers[6] = m7;
        times = new double[MAX_AMOUNT_MARKERS];
        setTimesMax();

        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
        times[4] = time5;
        times[5] = time6;
        times[6] = time7;
    }

    /**
     * Fill the times to the maximum value possible, so that unused InsertMarker slots do not create new threads
     */
    private void setTimesMax(){
        Arrays.fill(times, Double.MAX_VALUE);
    }

    /**
     * Get the full list of markers
     * @return all of the markers
     */
    public NeoInsertMarker[] getMarkers(){
        return markers;
    }

    /**
     * Get the maximum amount of markers allowed
     * @return the maximum amount of markers
     */
    public int getMAX_AMOUNT_MARKERS(){
        return MAX_AMOUNT_MARKERS;
    }

    /**
     * Get the InsertMarker at position i
     * @param i the index to get the InsertMarker in the markers list
     * @return The InsertMarker alias at i
     */
    public NeoInsertMarker getNeoInsertMarker(int i){
        return markers[i];
    }

    /**
     * Get the starting time for the marker at position i
     * @param i the index to get the time in the times list
     * @return the starting time for marker i
     */
    public double getTime(int i){
        return times[i];
    }

}
