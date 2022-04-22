package org.firstinspires.ftc.teamcode.auto;
/**
 * Program to take linear velocities from each wheel and translate
 * them into diffy pod velocities and then into encoder speeds.
 * Created by author Tiernan Lindauer for FTC team 7797.
 * @license Creative Commons
 * Last edited 4/18/22
 */

import java.util.ArrayList;

public class PathSequence {

    private ArrayList<NeoPath> trajectory;
    private Drivetrain drive;

    private double resolution;

    private ArrayList<Double> leftBottom;
    private ArrayList<Double> leftTop;

    private ArrayList<Double> rightBottom;
    private ArrayList<Double> rightTop;

    private double wheelRadius;
    private double encoderTicksRev;

    public PathSequence(Drivetrain dt, ArrayList<NeoPath> d, double wheelR){
        trajectory = d;
        wheelRadius = wheelR;
        drive = dt;

        resolution = 0.01;
        leftBottom = new ArrayList<Double>();
        leftTop = new ArrayList<Double>();
        rightBottom = new ArrayList<Double>();
        rightTop = new ArrayList<Double>();
    }
    public PathSequence(Drivetrain dt, ArrayList<NeoPath> d, double wheelR, double r){
        trajectory = d;
        resolution = r;
        wheelRadius = wheelR;
        drive = dt;

        leftBottom = new ArrayList<Double>();
        leftTop = new ArrayList<Double>();
        rightBottom = new ArrayList<Double>();
        rightTop = new ArrayList<Double>();
        encoderTicksRev = 537.7;
    }
    public PathSequence(Drivetrain dt, ArrayList<NeoPath> d, double wheelR, double r, double tpr){
        trajectory = d;
        resolution = r;
        wheelRadius = wheelR;
        drive = dt;

        leftBottom = new ArrayList<Double>();
        leftTop = new ArrayList<Double>();
        rightBottom = new ArrayList<Double>();
        rightTop = new ArrayList<Double>();
        encoderTicksRev = tpr;
    }

    /**
     * @Precondition the arraylist of paths has at least one path
     * @Postcondition sets the angular velocity of each of the drive pods (not geared) in radians per second
     */
    protected double convert(double v){
        v/=wheelRadius; // convert to angular velocity by radius
        v/=(2*3.14159);
        v*=encoderTicksRev;
        return v;
    }

    public void buildAll(){
        for(NeoPath p : trajectory) {
            p.build();
            double time = p.getExecuteTime();
            for(double i = 0;i < time;i+=resolution){
                double leftVel = p.getLeftVelocity(i);
                double rightVel = p.getRightVelocity(i);

                leftTop.add(encoderTicksRev*leftVel/(2*wheelRadius));
                leftBottom.add(encoderTicksRev*-leftVel/(2*wheelRadius));

                rightTop.add(encoderTicksRev*rightVel/(2*wheelRadius));
                rightBottom.add(encoderTicksRev*-rightVel/(2*wheelRadius));
            }
        }
    }
    public void build(int i){
        trajectory.get(i).build();
    }

    public void follow(){
        KalmanFilter k1 = new KalmanFilter(10,10), k2 = new KalmanFilter(10,10), k3 = new KalmanFilter(10,10), k4 = new KalmanFilter(10,10);
        PIDController p1 = new PIDController(0.3,0.3,0.8), p2 = new PIDController(0.3,0.3,0.8), p3 = new PIDController(0.3,0.3,0.8), p4 = new PIDController(0.3,0.3,0.8);
        PIDKController pk1 = new PIDKController(k1,p1), pk2 = new PIDKController(k2,p2), pk3 = new PIDKController(k2,p2), pk4 = new PIDKController(k2,p2);

        int index = 0;
        long startTime = System.currentTimeMillis();
        while(index < leftTop.size()){
            double leftVelocity = (drive.leftTop.getVelocity()+drive.leftBottom.getVelocity())/2.0;
            double rightVelocity = (drive.rightTop.getVelocity()+drive.rightBottom.getVelocity())/2.0;

            double leftTopTarget = encoderTicksRev*leftVelocity/(2*wheelRadius);
            double leftBottomTarget = encoderTicksRev*-leftVelocity/(2*wheelRadius);

            double rightTopTarget = encoderTicksRev*rightVelocity/(2*wheelRadius);
            double rightBottomTarget = encoderTicksRev*-rightVelocity/(2*wheelRadius);

            drive.leftTop.setVelocity(pk1.update((long)leftTopTarget, (long)drive.leftTop.getVelocity()));
            drive.leftBottom.setVelocity(pk2.update((long)leftBottomTarget, (long)drive.leftBottom.getVelocity()));
            drive.rightTop.setVelocity(pk2.update((long)rightTopTarget, (long)drive.rightTop.getVelocity()));
            drive.rightBottom.setVelocity(pk2.update((long)rightBottomTarget, (long)drive.rightBottom.getVelocity()));

            index++;

            long t = System.currentTimeMillis();

            //hold loop until it's time to move on
            while(t-startTime < index*resolution){
                t = System.currentTimeMillis();
            }
        }
    }

    public ArrayList<Double> getLeftBottom(){
        return leftBottom;
    }
    public ArrayList<Double> getLeftTop(){
        return leftTop;
    }
    public ArrayList<Double> getRightBottom(){
        return rightBottom;
    }
    public ArrayList<Double> getRightTop(){
        return rightTop;
    }
}
