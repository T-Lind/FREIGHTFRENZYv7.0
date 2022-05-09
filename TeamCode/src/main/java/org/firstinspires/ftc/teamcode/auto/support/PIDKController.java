package org.firstinspires.ftc.teamcode.auto.support;

/**
 * Program that combines both a kalman filter and PID loop.
 * Kalman filter returns what it predicts the actual system state
 * to be and the PID loop issues a correction based on that.
 * Created by @author Tiernan Lindauer for FTC team 7797.
 * @license Creative Commons
 * Last edited 4/26/22
 *
 * NOT USED as of 5/5/22.
 */

public class PIDKController {
    private KalmanFilter predictor;
    private PIDController corrector;

    public PIDKController(KalmanFilter k, PIDController p){
        predictor = k;
        corrector = p;
    }
    public double update(long target, long state){
        long predictedState = (long)predictor.filter(state);
        return state+corrector.update(target, predictedState);
    }
    public KalmanFilter getPredictor(){
        return predictor;
    }
    public PIDController getCorrector(){
        return corrector;
    }
}