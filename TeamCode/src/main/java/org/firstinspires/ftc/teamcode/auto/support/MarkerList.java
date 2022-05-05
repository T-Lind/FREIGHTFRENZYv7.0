package org.firstinspires.ftc.teamcode.auto.support;

import java.util.ArrayList;

public class MarkerList {
    ArrayList<InsertMarker> markers = new ArrayList<InsertMarker>();
    double[] times;

    public MarkerList(InsertMarker m, double[] t){
        markers.add(m);
        times = t;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, double[] t){
        markers.add(m);
        markers.add(m2);
        times = t;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, double[] t){
        markers.add(m);
        markers.add(m2);
        markers.add(m3);
        times = t;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, double[] t){
        markers.add(m);
        markers.add(m2);
        markers.add(m3);
        markers.add(m4);
        times = t;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, InsertMarker m5, double[] t){
        markers.add(m);
        markers.add(m2);
        markers.add(m3);
        markers.add(m4);
        markers.add(m5);
        times = t;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, InsertMarker m5, InsertMarker m6, double[] t){
        markers.add(m);
        markers.add(m2);
        markers.add(m3);
        markers.add(m4);
        markers.add(m5);
        markers.add(m6);
        times = t;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, InsertMarker m5, InsertMarker m6, InsertMarker m7, double[] t){
        markers.add(m);
        markers.add(m2);
        markers.add(m3);
        markers.add(m4);
        markers.add(m5);
        markers.add(m6);
        markers.add(m7);
        times = t;
    }
    public MarkerList(InsertMarker m,InsertMarker m2, InsertMarker m3, InsertMarker m4, InsertMarker m5, InsertMarker m6, InsertMarker m7, InsertMarker m8, double[] t){
        markers.add(m);
        markers.add(m2);
        markers.add(m3);
        markers.add(m4);
        markers.add(m5);
        markers.add(m6);
        markers.add(m7);
        markers.add(m8);
        times = t;
    }
    public ArrayList<InsertMarker> getMarkers(){
        return markers;
    }
    public InsertMarker getInsertMarker(int i){
        return markers.get(i);
    }
    public double getTime(int i){
        return times[i];
    }
}
