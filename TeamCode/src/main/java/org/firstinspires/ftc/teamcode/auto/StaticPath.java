package org.firstinspires.ftc.teamcode.auto;

/**
 * A program to take a path represented by an equation
 * and decompose it to motor power values on a differential swerve.
 * One thing to note that needs to be added is that
 * it needs to map the velocity to encoder ticks
 *
 * Also it needs to rescale the velocity vectors if they're too large.
 *
 * @author Tiernan Lindauer
 */

import java.util.ArrayList;

class StaticPath{
    double timeStep;

    public StaticPath(){

    }

    public StaticPath(double ts){
        timeStep = ts;
    }

    public void build(){
        timeStep = 0;
    }
}

class SplineStaticHeading extends StaticPath{
    // specified in the trajectory
    private double executeTime;

    // all empty to start out with
    private ArrayList<Double> velocity;
    private ArrayList<Double> angle;
    private ArrayList<Double> angularVelocity;
    private ArrayList<Double> motorOneVelocity;
    private ArrayList<Double> motorTwoVelocity;


    // two different constructors, staticAngle is only for splining
    public SplineStaticHeading(double executeT, double step_time){
        executeTime = executeT;
        timeStep = step_time;

        velocity = new ArrayList<Double>();
        angle = new ArrayList<Double>();
        angularVelocity = new ArrayList<Double>();
        motorOneVelocity = new ArrayList<Double>();
        motorTwoVelocity = new ArrayList<Double>();
    }

    // the default movement function
    public double f(double t){
        return 0;
    }


    public void buildStaticAngle(){
        for(int i=0;i<(1/timeStep)*executeTime-1;i++){
            double slope = (f((i+1)*timeStep)-f(i*timeStep))/timeStep;
            angle.add(Math.atan(slope));
        }
        for(int i=0;i<angle.size()-1;i++)
            angularVelocity.add((angle.get(i+1)-angle.get(i))/timeStep);

    }
    public void buildStaticVelocity() {
        for (int i = 0; i < (1 / timeStep) * executeTime - 2; i++)
            velocity.add(Math.sqrt(timeStep * timeStep + (f((i + 1) * timeStep) - f(i * timeStep)) * (f((i + 1) * timeStep) - f(i * timeStep))) / timeStep);
    }

    public void motorOnePower() {
        // need to map these velocities to diffy swerve encoder velocities
        for (int i = 0; i < angularVelocity.size(); i++) {
            double[] s = new double[2];
            s[0] = velocity.get(i);
            s[1] = angularVelocity.get(i);

            motorOneVelocity.add((s[0] + (s[1] - s[0]) / 2) * Math.sqrt(2));
        }
    }
    public void motorTwoPower() {
        for (int i = 0; i < angularVelocity.size(); i++) {
            double[] s = new double[2];
            s[0] = velocity.get(i);
            s[1] = angularVelocity.get(i);

            double mag = (s[1] - s[0]) / 2;
            mag *= mag;
            if (s[1] - s[0] < 0)
                mag *= -1;

            motorTwoVelocity.add(mag);
        }
    }

    public ArrayList<Double> getMotorOneVelocity(){
        return motorOneVelocity;
    }
    public ArrayList<Double> getMotorTwoVelocity(){
        return motorTwoVelocity;
    }


    @Override
    public void build() {
        buildStaticVelocity();
        buildStaticAngle();
        motorOnePower();
        motorTwoPower();
    }
}