package org.firstinspires.ftc.teamcode.auto.support;

/**
 * Supporting file to create a static array of markers.
 * @author Tiernan Lindauer
 * For team 7797 Victorian Voltage.
 */
public class MarkerList {
    InsertMarker[] markers;

    /**
     * All of these methods are just meant to nicely create a static array of the InsertMarker implementation.
     * @param m the marker (listed m, m2, m3, etc.)
     */
    public MarkerList(InsertMarker m){
        markers = new InsertMarker[1];
        markers[0] = m;
    }
    public MarkerList(InsertMarker m,InsertMarker m2){
        markers = new InsertMarker[2];
        markers[0] = m;
        markers[1] = m2;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3){
        markers = new InsertMarker[3];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4){
        markers = new InsertMarker[4];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, InsertMarker m5){
        markers = new InsertMarker[5];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, InsertMarker m5, InsertMarker m6){
        markers = new InsertMarker[6];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        markers[5] = m6;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, InsertMarker m5, InsertMarker m6, InsertMarker m7){
        markers = new InsertMarker[7];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        markers[5] = m6;
        markers[6] = m7;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, InsertMarker m5, InsertMarker m6, InsertMarker m7, InsertMarker m8){
        markers = new InsertMarker[8];
        markers[0] = m;
        markers[1] = m2;
        markers[2] = m3;
        markers[3] = m4;
        markers[4] = m5;
        markers[5] = m6;
        markers[6] = m7;
        markers[7] = m8;
    }
    public InsertMarker[] getMarkers(){
        return markers;
    }

    public InsertMarker getInsertMarker(int i){
        return markers[i];
    }
}
