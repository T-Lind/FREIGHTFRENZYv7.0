package org.firstinspires.ftc.teamcode.auto;
import java.util.ArrayList;

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

class Path{
    private double executeTime;
    private double timeStep;

    private ArrayList<Double> velocity;
    private ArrayList<Double> angle;
    private ArrayList<Double> angularVelocity;
    private ArrayList<Double> motorOneVelocity;
    private ArrayList<Double> motorTwoVelocity;
    public Path(double executeT, double step_time){
        executeTime = executeT;
        timeStep = step_time;

        velocity = new ArrayList<Double>();
        angle = new ArrayList<Double>();
        angularVelocity = new ArrayList<Double>();
        motorOneVelocity = new ArrayList<Double>();
        motorTwoVelocity = new ArrayList<Double>();
    }

    public double getExecuteTime(){
        return executeTime;
    }
    public double getTimeStep(){
        return timeStep;
    }

    public double f(double t){
        return 0;
    }


    public void buildAngle(){
        for(int i=0;i<(1/timeStep)*executeTime-1;i++){
            double slope = (f((i+1)*timeStep)-f(i*timeStep))/timeStep;
            angle.add(Math.atan(slope));
        }
        for(int i=0;i<angle.size()-1;i++)
            angularVelocity.add((angle.get(i+1)-angle.get(i))/timeStep);

    }
    public void buildVelocity() {
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

    public void build(){
        buildVelocity();
        buildAngle();
        motorOnePower();
        motorTwoPower();
    }
}

class pathOne extends Path{
    public pathOne(double executeT, double time_step) {
        super(executeT, time_step);
    }

    @Override
    public double f(double t){
        return -1*Math.cos(3.14159*t)+1;
    }
}

public class pathingtest {

    public static ArrayList<Double> getVelocity(Path p) {
        ArrayList<Double> velocity = new ArrayList<Double>();
        double timeStep = p.getTimeStep();

        for (int i = 0; i < (1 / timeStep) * p.getExecuteTime() - 2; i++)
            velocity.add(Math.sqrt(timeStep * timeStep + (p.f((i + 1) * timeStep) - p.f(i * timeStep)) * (p.f((i + 1) * timeStep) - p.f(i * timeStep))) / timeStep);

        return velocity;
    }

    public static ArrayList<Double> motorOnePower(ArrayList<Double> angularVelocities, ArrayList<Double> velocities) {
        // need to map these velocities to diffy swerve encoder velocities

        ArrayList<Double> motorOneVelocity = new ArrayList<Double>();
        for (int i = 0; i < angularVelocities.size(); i++) {
            double[] s = new double[2];
            s[0] = velocities.get(i);
            s[1] = angularVelocities.get(i);

            motorOneVelocity.add((s[0] + (s[1] - s[0]) / 2) * Math.sqrt(2));
        }
        return motorOneVelocity;
    }

    public static ArrayList<Double> motorTwoPower(ArrayList<Double> angularVelocities, ArrayList<Double> velocities) {
        ArrayList<Double> motorTwoVelocity = new ArrayList<Double>();
        for (int i = 0; i < angularVelocities.size(); i++) {
            double[] s = new double[2];
            s[0] = velocities.get(i);
            s[1] = angularVelocities.get(i);

            double mag = (s[1] - s[0]) / 2;
            mag *= mag;
            if (s[1] - s[0] < 0)
                mag *= -1;

            motorTwoVelocity.add(mag);
        }
        return motorTwoVelocity;
    }


    public static void main(String[] args) {
        final double timeStep = 0.01; // in seconds

        Path trajectory1 = new pathOne(1,timeStep);
        trajectory1.build();

        ArrayList<Double> m1p = trajectory1.getMotorOneVelocity();
        ArrayList<Double> m2p = trajectory1.getMotorTwoVelocity();

        System.out.println(m1p);
        System.out.println(m2p);
    }
}
