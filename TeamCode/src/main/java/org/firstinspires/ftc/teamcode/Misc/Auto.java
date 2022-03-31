package org.firstinspires.ftc.teamcode.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//import org.firstinspires.ftc.teamcode.Camera;


//@Autonomous
public abstract class Auto extends LinearOpMode {
    //test pushing
    private ElapsedTime runtime;
    private Robot r;
    @Override
    public void runOpMode() throws InterruptedException {
        runtime = new ElapsedTime();

        r = new Robot();
        r.moveRobot();

    }
    public ElapsedTime returnRuntime() throws InterruptedException {
        return runtime;
    }

    public void  heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
        telemetry.update();
    }
}
