package org.firstinspires.ftc.teamcode.auto;
import java.util.ArrayList;

/**
 * Runner of StaticPath and SplinePath
 */

class PathOne extends StaticPath{
    public PathOne(double executeT, double time_step) {
        super(executeT, time_step);
    }

    @Override
    public double f(double t){
        return -1000*Math.cos(3.14159/1000*t)+1000;
    }

}
class PathTwo extends StaticPath{
    public PathTwo(double executeT, double time_step) {
        super(executeT, time_step);
    }

    @Override
    public double f(double t){
        return t;
    }

}

public class PathingTest {
    public static void main(String[] args) {
        final double timeStep = 0.01; // in seconds

        StaticPath trajectory1 = new PathOne(10,timeStep);
        trajectory1.build();


        ArrayList<Double> m1p = trajectory1.getMotorOneVelocity();
        ArrayList<Double> m2p = trajectory1.getMotorTwoVelocity();

//        System.out.println(m1p);
//        System.out.println(m2p);

        double[] radii = new double[2];
        radii[0] = 60;
        radii[1] = 30;
        double[] arcLengths = new double[2];
        arcLengths[0] = -3.14159/4;
        arcLengths[1] = 3.14159/2;

        SplinePath trajectory2 = new SplinePath(12.0,10, radii, arcLengths);
        trajectory2.build();

        for(int i=0;i<60;i++)
            System.out.println(trajectory2.getLeftVelocity((double)i/2)+" "+trajectory2.getRightVelocity((double)i/2));
    }
}