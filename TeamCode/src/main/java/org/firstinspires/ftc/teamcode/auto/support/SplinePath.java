package org.firstinspires.ftc.teamcode.auto.support;
/**
 * Translates a series of circle radii/arc lengths into motor power values for a 2 wheeled robot.
 * created by @author Tiernan Lindauer for FTC team 7797.
 * @license  MIT License
 * Last edited 5/1/22
 *
 */

import java.util.ArrayList;

public class SplinePath extends NeoPath {
    private double[] radii;
    private double[] arcLengths;
    private double velocity;
    private double trackWidth;
    private double accelerationTime;
    private double TpA;
    private double TpD;
    private ArrayList<Double> times;

    private KalmanFilter k;
    private KalmanFilter k2;
    private PIDController p;
    private PIDController p2;

    // track width is how far the wheels are apart, r is the radius of each of the turns, v is an ArrayList of static arrays of the velocities.
    /**
     * @Precondition each static array in v is 3 long, size of r and v is equal
     * @Convention positive arc length is CCW, negative is CW
     * **/
    public SplinePath(double tw, double v, double accTime, double[] r, double[] al){
        trackWidth = tw;
        radii = r;
        arcLengths = al;
        velocity = v;
        accelerationTime = accTime;
        TpA = 0;
        TpD = 0;

        times = new ArrayList<Double>();

        k = new KalmanFilter(0);
        k2 = new KalmanFilter(0);
        p = new PIDController(0);
        p2 = new PIDController(0);
    }

    @Override
    public void build(){
        for(int i=1;i<arcLengths.length-1;i++)
            arcLengths[i]*=(2*3.14159265);
        TpA = (3*Math.abs(arcLengths[0])-2*velocity*accelerationTime)/(3*velocity);
        TpD = (Math.abs(arcLengths[arcLengths.length-1])-((velocity*accelerationTime)/3))/velocity;
        System.out.println("TpA, TpD: "+TpA+" "+TpD);
        times.add(accelerationTime+TpA);
        for(int i=1;i<arcLengths.length-1;i++)
            times.add(times.get(times.size()-1)+Math.abs(arcLengths[i])*velocity);
        times.add(times.get(times.size()-1)+accelerationTime+TpD);
        setBuilt(true);
    }

    @Override
    public double getExecuteTime(){
        return times.get(times.size()-1);
    }

    public ArrayList<Double> getTimes(){
        return times;
    }

    public int getArc(double t){
        for(int i=0;i<times.size();i++)
            if(t < times.get(i))
                return i;
        return -1;
    }
    public double getVelocity(double t){
        int arc = getArc(t);
        if(arc == -1)
            return 0;

        if(arc > 0 && arc < arcLengths.length-1) {
            return velocity;
        }
        else if(arc == 0){
            if(t < accelerationTime){
                return velocity*Math.sqrt(t/accelerationTime);
            }
            return velocity;
        }
        else {
            if(t < times.get(times.size()-2)+TpD)
                return velocity;
            return velocity-velocity*Math.sqrt((t-times.get(times.size()-2)-TpD)/accelerationTime);
        }
    }
    @Override
    public double getLeftVelocity(double t){
        if(getArc(t) == -1) {
            this.setCompleted(true);
            return 0;
        }
        double v = getVelocity(t);
        if(arcLengths[getArc(t)] > 0)
            v = v-(v*trackWidth/(2*radii[getArc(t)]));
        else
            v = v+(v*trackWidth/(2*radii[getArc(t)]));
        v*=-1;
        return v;
    }
    @Override
    public double getRightVelocity(double t){
        if(getArc(t) == -1) {
            this.setCompleted(true);
            return 0;
        }
        double v = getVelocity(t);
        if(arcLengths[getArc(t)] > 0)
            return v+(v*trackWidth/(2*radii[getArc(t)]));
        return v-(v*trackWidth/(2*radii[getArc(t)]));
    }
//    public double[] update(double left, double right, double time){
//        left = convert(left);
//        right = convert(right);
//
//        left = k.filter(left);
//        right = k2.filter(right);
//
//        double l = getLeftVelocity(time);
//        double r = getRightVelocity(time);
//
//        double corL = p.update((long)l, (long)left);
//        double corR = p.update((long)r, (long)right);
//
//        double[] ret = new double[2];
//        ret[0] = corL+left;
//        ret[1] = corR+right;
//        return ret;
//    }

}

