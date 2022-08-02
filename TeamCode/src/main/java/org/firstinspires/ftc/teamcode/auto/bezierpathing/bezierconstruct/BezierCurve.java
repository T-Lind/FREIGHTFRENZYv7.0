package org.firstinspires.ftc.teamcode.auto.bezierpathing.bezierconstruct;

import java.util.ArrayList;

public class BezierCurve {
    private BezierCurveStorage storage;
    private ArrayList<Point> control_points;

    private double timeStepResolution;

    public BezierCurve(ArrayList<Point> control_points) {
        this.control_points = control_points;
        timeStepResolution = 0.001;

        storage = new BezierCurveStorage();

//        for(double time=0;time<1;time+=timeStepResolution) {
//            storage.pointsInCurve.add(BezierCurveStorage.recursiveLerp(this.control_points, time));
//        }
    }

    public Point getCurrentPoint(double time) {
        return BezierCurveStorage.recursiveLerp(control_points, time);
    }
}
