package org.firstinspires.ftc.teamcode.auto;

import java.util.ArrayList;

class Line extends NeoPath{
    private double distance;
    private double maxVelocity;
    private double maxAcceleration;
    private double eT;

    private ArrayList<double[]> velocities;
    private ArrayList<Double> motorVelocities;


    public Line(double d, double v, double acc){
        distance = d;
        maxVelocity = v;
        maxAcceleration = acc;

        velocities = new ArrayList<double[]>();
        eT = 0;
    }

    @Override
    public void build(){
        eT = distance/maxVelocity;
        while(2*maxVelocity/eT > maxAcceleration){
            maxVelocity -= (maxVelocity/50);
            eT = distance/maxVelocity;
        }
        double[] p1 = {0,0};
        double[] p2 = {eT/2, maxVelocity};
        double[] p3 = {eT,0};

        velocities.add(p1);
        velocities.add(p2);
        velocities.add(p3);

    }

    @Override
    public double getExecuteTime(){
        return eT;
    }

    public double getVelocity(double t){
        if(t < eT/2)
            return (2*maxVelocity/eT)*t;
        if(t <= eT)
            return -2*maxVelocity/eT*(t-eT/2)+maxVelocity;
        return 0;
    }

    @Override
    public double getLeftVelocity(double t){
        return getVelocity(t);
    }
    @Override
    public double getRightVelocity(double t){
        return getVelocity(t);
    }
}