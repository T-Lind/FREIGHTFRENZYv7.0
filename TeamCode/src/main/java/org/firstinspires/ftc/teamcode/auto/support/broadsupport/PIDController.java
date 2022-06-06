package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

/**
 * A PID control loop for general purpose,
 * created by
 * @author Tiernan Lindauer
 * for FTC team 7797.
 */

import java.util.ArrayList;
public class PIDController {
    private double proportional;
    private double integral;
    private double derivative;
    private int forgetLength;

    private ArrayList<Long> data;
    private ArrayList<Long> time;

     /**
     * Default constructor - assigns moderate values to the PID.
     */
    public PIDController(){
        proportional = 0.3;
        integral = 0.3;
        derivative = 0.8;
        data = new ArrayList<Long>();
        time = new ArrayList<Long>();
        forgetLength = 64;
    }
    /**
     * DeviceCode constructor
     * @param deviceCode corresponds to what the Kalman Filter is for to reduce code and code errors.
     *                   When deviceCode is 0, it is for motors
     *                   deviceCode 1 IS NOT USED IN PID - FOR DISTANCE SENSORS
     */
    public PIDController(int deviceCode){
        if(deviceCode == 0){
            proportional = 0.4;
            integral = 0.5;
            derivative = 0.3;
            data = new ArrayList<Long>();
            time = new ArrayList<Long>();
            forgetLength = 64;
        }
        if(deviceCode == 2){
            proportional = 0.01;
            integral = 0.01;
            derivative = 0.01;
            data = new ArrayList<Long>();
            time = new ArrayList<Long>();
            forgetLength = 64;
        }
    }
    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     */
    public PIDController(double Kp, double Ki, double Kd) {
        proportional = Kp;
        integral = Ki;//*1000;
        derivative = Kd;
        data = new ArrayList<Long>();
        time = new ArrayList<Long>();
        forgetLength = 64;
    }
    /**
     * construct PID controller
     * @param Kp Proportional coefficient
     * @param Ki Integral coefficient
     * @param Kd Derivative coefficient
     * @param len is the amount of data entries after which they are not included in the PID calculations.
     */
    public PIDController(double Kp, double Ki, double Kd, int len) {
        proportional = Kp;
        integral = Ki*1000;
        derivative = Kd;
        forgetLength = len;
        data = new ArrayList<Long>();
        time = new ArrayList<Long>();
        forgetLength = len;
    }

    /**
     * A private method to compute the integral term.
     * @return a right Riemann sum of the error so far.
     */
    private long getIntegral(){
        long sum = 0;
        if(time.size() > 2){
            for(int i=0;i<time.size()-1;i++)
                sum += ((time.get(i + 1) - time.get(i))/160.0 * ((data.get(i + 1) + data.get(i)) / 2.0));
        }
        return sum;
    }

    /**
     * update the PID controller output
     * @param target where we would like to be, also called the reference
     * @param state where we currently are, I.E. motor position
     * @return the command to our motor, I.E. motor power
     */
    public double update(long target, long state) {
        // PID logic and then return the output
        time.add(System.currentTimeMillis());
        long error = target-state;
        data.add(error);

        if(data.size() < forgetLength+1)
            return proportional*error;
        else{
            long dNum = data.get(data.size()-1)-data.get(data.size()-forgetLength);
            long dDen = time.get(time.size()-1)-time.get(time.size()-forgetLength);
            //dDen*=-1000;
            if(dDen==0)
                dDen = (long)1.1;
            return proportional * error + integral * getIntegral();// + derivative * dNum / dDen;
        }
    }
}