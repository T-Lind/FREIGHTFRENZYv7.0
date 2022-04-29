package org.firstinspires.ftc.teamcode.auto;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

/**
 * Runner of StaticPath and SplinePath
 * created by @author Tiernan Lindauer for FTC team 7797.
 * @license  Creative Commons
 * Last edited 4/26/22
 *
 */

class PathOne extends SplineStaticHeading{
    public PathOne(double executeT, double time_step) {
        super(executeT, time_step);
    }

    @Override
    public double f(double t){
        return -1000*Math.cos(3.14159/1000*t)+1000;
    }

}
class PathTwo extends SplineStaticHeading{
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
        /*
          Code to follow a spline path (rotating wheels not bot)
         */
//        final double timeStep = 0.01; // in seconds
//
//        SplineStaticHeading trajectory1 = new PathOne(10,timeStep);
//        trajectory1.build();
//
//
//        ArrayList<Double> m1p = trajectory1.getMotorOneVelocity();
//        ArrayList<Double> m2p = trajectory1.getMotorTwoVelocity();

//        System.out.println(m1p);
//        System.out.println(m2p);


        /*
          Code to follow a spline path by moving the bot - differential velocity
          **/
//        double[] radii = new double[2];
//        radii[0] = 60;
//        radii[1] = 30;
//        double[] arcLengths = new double[2];
//        arcLengths[0] = -3.14159/4;
//        arcLengths[1] = 3.14159/2;
//
//        SplinePath trajectory2 = new SplinePath(12.0,10,0.5, radii, arcLengths);
//        trajectory2.build();

//        for(int i=0;i<60;i++)
//            System.out.println(trajectory2.getLeftVelocity((double)i/2)+" "+trajectory2.getRightVelocity((double)i/2));

        /*
          Code to move forward while first speeding up and then slowing down (velocity curve)
          Must specify a distance to travel, velocity, and acceleration.
          Always obeys max v/a at the expense of time.
          This can be used in combination with a turn before to essentially execute a lineTo.
         */
//
//        Line trajectory3 = new Line(1,0.3);
//        trajectory3.build();


    }
}