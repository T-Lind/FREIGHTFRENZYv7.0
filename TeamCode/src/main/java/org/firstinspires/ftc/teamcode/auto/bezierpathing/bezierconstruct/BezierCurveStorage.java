package org.firstinspires.ftc.teamcode.auto.bezierpathing.bezierconstruct;

import java.util.ArrayList;

public class BezierCurveStorage {
    public ArrayList<Point> pointsInCurve;
    public ArrayList<Double> rawDerivative;
    public ArrayList<Point> derivativeComponentStorage;
    public ArrayList<Double> velocity;
    public ArrayList<Double> angle;

    public static Point lerp(Point point0, Point point1, double t) {
        double x = (1 - t) * point0.x + t * point1.x;
        double y = (1 - t) * point0.y + t * point1.y;

        return new Point(x, y);
    }

    public static Point recursiveLerp(ArrayList<Point> points, double time) {
        if(points.size() == 2) {
            return lerp(points.get(0), points.get(1), time);
        }

        int index = 0;
        ArrayList<Point> listOfLerps = new ArrayList<>();
        while(index < points.size()-1) {
            Point point1 = lerp(points.get(index), points.get(index+1), time);
            listOfLerps.add(point1);
            index += 1;
        }
        return recursiveLerp(listOfLerps, time);
    }

    public static int binarySearch(ArrayList<Double> arr, int l, int r, int x)
    {
        if (r>=l)
        {
            int mid = l + (r - l)/2;

            // If the element is present at the
            // middle itself
            if (arr.get(mid-1) <= x && x <= arr.get(mid+1))
                return mid;

            // If element is smaller than mid, then
            // it can only be present in left subarray
            if (arr.get(mid) > x)
                return binarySearch(arr, l, mid-1, x);

            // Else the element can only be present
            // in right subarray
            return binarySearch(arr, mid+1, r, x);
        }

        // We reach here when element is not present
        //  in array
        return -1;
    }

    public static ArrayList<Double> getX(ArrayList<Point> points) {
        ArrayList<Double> xVals = new ArrayList<>();
        for(Point point : points) {
            xVals.add(point.x);
        }
        return xVals;
    }

    public static ArrayList<Double> getY(ArrayList<Point> points) {
        ArrayList<Double> yVals = new ArrayList<>();
        for(Point point : points) {
            yVals.add(point.y);
        }
        return yVals;
    }
}
