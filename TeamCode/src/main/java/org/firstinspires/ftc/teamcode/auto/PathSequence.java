package org.firstinspires.ftc.teamcode.auto;

import java.nio.file.Path;
import java.util.ArrayList;

public class PathSequence {

    private ArrayList<NeoPath> trajectory;
    private double resolution;

    private ArrayList<Double> leftBottom;
    private ArrayList<Double> leftTop;

    private ArrayList<Double> rightBottom;
    private ArrayList<Double> rightTop;

    public PathSequence(ArrayList<NeoPath> d){
        trajectory = d;
        resolution = 0.01;
        leftBottom = new ArrayList<Double>();
        leftTop = new ArrayList<Double>();
        rightBottom = new ArrayList<Double>();
        rightTop = new ArrayList<Double>();
    }
    public PathSequence(ArrayList<NeoPath> d, double r){
        trajectory = d;
        resolution = r;
        leftBottom = new ArrayList<Double>();
        leftTop = new ArrayList<Double>();
        rightBottom = new ArrayList<Double>();
        rightTop = new ArrayList<Double>();
    }

    public void buildAll(){
        for(NeoPath p : trajectory) {
            p.build();
            double time = p.getExecuteTime();
            for(double i = 0;i < time;i+=resolution){
                double leftVel = p.getLeftVelocity(i);
                double rightVel = p.getRightVelocity(i);

                leftTop.add(leftVel/2);
                leftBottom.add(-leftVel/2);

                rightTop.add(rightVel/2);
                rightBottom.add(-rightVel/2);
            }
        }
    }
    public void build(int i){
        trajectory.get(i).build();
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
