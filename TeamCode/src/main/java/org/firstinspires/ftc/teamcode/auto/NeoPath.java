package org.firstinspires.ftc.teamcode.auto;

public class NeoPath {
    private double executeTime = 0;

    public void build(){
        return;
    }

    public void setExecuteTime(double d){
        executeTime = d;
    }
    public double getExecuteTime(){
        return executeTime;
    }

    public double getLeftVelocity(double t){
        return 0;
    }
    public double getRightVelocity(double t){
        return 0;
    }
}
