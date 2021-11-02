package org.firstinspires.ftc.teamcode;

//import org.firstinspires.ftc.teamcode.Camera;

//import gayness

public class Robot extends Auto {
    private Movement m;

    public Robot(){
        m = new Movement(hardwareMap);

    }

    public void moveRobot() throws InterruptedException{
        m.initializeMotors();
        waitForStart();
        //m.moveByWheelEncoders(0, 10, 0.5, "straight");
    }



}