package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.ConstantHeadingSpline;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Line;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Path;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequence;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Robot;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Turn;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Drivetrain;

import java.util.ArrayList;

@Autonomous(name="ConstantHeadingSplineTest")
public class ConstantHeadingSplineTest extends Robot {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        // Robot is inherited, meaning that anything you put in Robot will be here as
        // well (as long as you make it protected), so is LinearOpMode (which BotContainer inherits)

        // Create the sequence of paths

        double[] radii = {1,0.35,-1.5};
        double[] arcLengths = {0.5,0.3,1};

        Path spline = new ConstantHeadingSpline(0.2, 0.2, radii, arcLengths);
        addPath(spline);

        initialize();

        // run the auto and any markers
        executeAuto();
    }
}
