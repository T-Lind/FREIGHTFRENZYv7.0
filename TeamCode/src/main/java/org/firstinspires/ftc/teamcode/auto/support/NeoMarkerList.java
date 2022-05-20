package org.firstinspires.ftc.teamcode.auto.support;

/**
 * Supporting file to create a static array of markers.
 * @author Tiernan Lindauer
 * For team 7797 Victorian Voltage.
 */
public class NeoMarkerList {
    InsertMarker[] markers;
    double times[];
    /**
     * All of these methods are just meant to nicely create a static array of the InsertMarker implementation.
     * @param m the marker (listed m, m2, m3, etc.)
     */
    public NeoMarkerList(InsertMarker m, double time1){
        markers = new InsertMarker[1];
        markers[0] = m;
        times = new double[1];
        times[0] = time1;
    }
    public NeoMarkerList(InsertMarker m, double time1, InsertMarker m2, double time2){
        markers = new InsertMarker[2];
        markers[0] = m;
        markers[1] = m2;
        times = new double[2];
        times[0] = time1;
        times[1] = time2;
    }
    public NeoMarkerList(InsertMarker m, double time1, InsertMarker m2, double time2,
                         InsertMarker m3, double time3){
        markers = new InsertMarker[3];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        times = new double[3];
        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
    }
    public NeoMarkerList(InsertMarker m, double time1, InsertMarker m2, double time2,
                         InsertMarker m3, double time3, InsertMarker m4, double time4){
        markers = new InsertMarker[4];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        times = new double[4];
        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
    }
    public NeoMarkerList(InsertMarker m, double time1, InsertMarker m2, double time2,
                         InsertMarker m3, double time3, InsertMarker m4, double time4,
                         InsertMarker m5, double time5){
        markers = new InsertMarker[5];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        times = new double[5];
        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
        times[4] = time5;
    }
    public NeoMarkerList(InsertMarker m, double time1, InsertMarker m2, double time2,
                         InsertMarker m3, double time3, InsertMarker m4, double time4,
                         InsertMarker m5, double time5, InsertMarker m6, double time6){
        markers = new InsertMarker[6];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        markers[5] = m6;
        times = new double[6];
        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
        times[4] = time5;
        times[5] = time6;
    }
    public NeoMarkerList(InsertMarker m, double time1, InsertMarker m2, double time2,
                         InsertMarker m3, double time3, InsertMarker m4, double time4,
                         InsertMarker m5, double time5, InsertMarker m6, double time6,
                         InsertMarker m7, double time7){
        markers = new InsertMarker[7];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        markers[5] = m6;
        markers[6] = m7;
        times = new double[7];
        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
        times[4] = time5;
        times[5] = time6;
        times[6] = time7;
    }
    public NeoMarkerList(InsertMarker m, double time1, InsertMarker m2, double time2,
                         InsertMarker m3, double time3, InsertMarker m4, double time4,
                         InsertMarker m5, double time5, InsertMarker m6, double time6,
                         InsertMarker m7, double time7, InsertMarker m8, double time8){
        markers = new InsertMarker[8];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        markers[5] = m6;
        markers[6] = m7;
        markers[7] = m8;
        times = new double[8];
        times[0] = time1;
        times[1] = time2;
        times[2] = time3;
        times[3] = time4;
        times[4] = time5;
        times[5] = time6;
        times[6] = time7;
        times[7] = time8;
    }
    public InsertMarker[] getMarkers(){
        return markers;
    }

    public InsertMarker getInsertMarker(int i){
        return markers[i];
    }

    public double getTime(int i){
        return times[i];
    }

    public int length(){
        return markers.length;
    }
}
