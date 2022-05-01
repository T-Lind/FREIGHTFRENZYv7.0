package org.firstinspires.ftc.teamcode.auto;

/**
 * A Kalman Filter for 1D data.
 * It can be used for multiple dimensioned data, however a separate
 * object must be created for each dimension.
 * Created for Victorian Voltage (FTC team 7797) 2022-2023 FIRST Tech Challenge Season/Preseason ONLY
 * 9527 Rogue Resistance or any other team may not use this code without permission from the author.
 *
 * Use this object in a runner like any other.
 *
 * @license Creative Commons
 *
 * Last edited 4/18/22
 *
 * @author Tiernan Lindauer
 */
public class KalmanFilter {

    // Vectors
    private double A = 1;
    private double B = 0;
    private double C = 1;

    // Noise assumptions
    private double R;
    private double Q;

    // Kalman filter return data
    private double cov = Double.NaN;
    private double x = Double.NaN;

    /*
    R is process noise
    Q is measurment noise
    A is the state vector
    B is the control vector
    C is the measurement vector
     */

    /**
     * Default constructor.
     * @param deviceCode corresponds to what the Kalman Filter is for to reduce code and code errors.
     *                   When deviceCode is 0, it is for motors
     *                   When deviceCode is 1, it is for distance sensors (in CM)
     *                   Additional deviceCodes will be added here.
     *                   This method is shared with the PIDController class.
     */
    public KalmanFilter(int deviceCode){
        if(deviceCode == 0){
            this.R = 7;
            this.Q = 4;
        }
        else if(deviceCode == 1){
            this.R = 2;
            this.Q = 10;
        }
    }

    public KalmanFilter(double R, double Q, double A, double B , double C){
        this.R = R;
        this.Q = Q;
        this.A = A;
        this.B = B;
        this.C = C;

        this.cov = Double.NaN;
        this.x = Double.NaN; // estimated signal without noise
    }

    // R is process noise, Q is measurement noise. No specified state/control/measurement vectors, set to default 1,0,1
    public KalmanFilter(double R, double Q){
        this.R = R;
        this.Q = Q;

    }

    // Filter a measurement: measured value is measurement, controlled input value is u.
    public double filter(double measurement, double u){

        if (Double.isNaN(this.x)) {
            this.x = (1 / this.C) * measurement;
            this.cov = (1 / this.C) * this.Q * (1 / this.C);
        }else {
            double predX = (this.A * this.x) + (this.B * u);
            double predCov = ((this.A * this.cov) * this.A) + this.R;

            // Kalman gain
            double K = predCov * this.C * (1 / ((this.C * predCov * this.C) + this.Q));

            // Correction
            this.x = predX + K * (measurement - (this.C * predX));
            this.cov = predCov - (K * this.C * predCov);
        }
        return this.x;
    }

    // Filter a measurement taken
    public double filter(double measurement){
        double u = 0;
        if (Double.isNaN(this.x)) {
            this.x = (1 / this.C) * measurement;
            this.cov = (1 / this.C) * this.Q * (1 / this.C);
        }else {
            double predX = (this.A * this.x) + (this.B * u);
            double predCov = ((this.A * this.cov) * this.A) + this.R;

            // Kalman gain
            double K = predCov * this.C * (1 / ((this.C * predCov * this.C) + this.Q));

            // Correction
            this.x = predX + K * (measurement - (this.C * predX));
            this.cov = predCov - (K * this.C * predCov);
        }
        return this.x;
    }


    // Return the last measurement taken
    public double lastMeasurement(){
        return this.x;
    }

    // Set measurement noise
    public void setMeasurementNoise(double noise){
        this.Q = noise;
    }

    // Set the process noise
    public void setProcessNoise(double noise){
        this.R = noise;
    }
}
