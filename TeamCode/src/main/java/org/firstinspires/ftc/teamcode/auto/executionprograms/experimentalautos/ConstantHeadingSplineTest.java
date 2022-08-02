package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.ConstantHeadingSpline;
import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.Path;
import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.Robot;

@Autonomous(name="ConstantHeadingSplineTest")
public class ConstantHeadingSplineTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        // Define the spline parts
        double[] radii = {1,0.35,-1.5};
        double[] arcLengths = {0.5,0.3,1};

        Path spline = new ConstantHeadingSpline(0.2, 0.2, radii, arcLengths);

        addPath(spline);

        // Initialize the robot
        initialize();

        // Run the auto and any markers
        executeAuto();
    }
}
