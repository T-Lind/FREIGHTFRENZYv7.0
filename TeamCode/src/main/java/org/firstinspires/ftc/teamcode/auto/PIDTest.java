package org.firstinspires.ftc.teamcode.auto;
/**
 * A test program to evaluate the PIDK controller.
 * @author Tiernan Lindauer created for FTC team 7797.
 * Last edited 4/18/22
 *
 */

import java.util.ArrayList;
public class PIDTest {
    public static void main(String[] args){
        KalmanFilter k = new KalmanFilter(10,10);
        PIDController pid = new PIDController(0.3,0.3,0.8);

        PIDKController pidController = new PIDKController(k,pid);
        ArrayList<Long> list = new ArrayList<Long>();
        ArrayList<Long> integral = new ArrayList<Long>();
        long state = 0;
        for(int i=0;i<100;i++){
            state += pidController.update(100, state);
            list.add(state);
        }
        System.out.println(list);
    }
}
