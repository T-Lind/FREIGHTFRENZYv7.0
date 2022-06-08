package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Timer;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.BotContainer;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoInsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoMarkerList;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name="AsynchTest")
public class AsynchTest extends BotContainer {

    @Override
    public void runOpMode() throws InterruptedException {
        // BotContainer is inherited, meaning that anything you put in BotContainer will be here as
        // well (as long as you make it protected), so is LinearOpMode (which BotContainer inherits)

        // Create the sequence of paths
//        ArrayList<NeoPath> sequenceInstructions = new ArrayList<>();
//        NeoPath line = new Line(0.5, 0.5);
//        sequenceInstructions.add(line);

        // Temporal Markers (Insert Markers) to execute during the path

        // Assign variable to not run the risk of repeating the marker code
        AtomicBoolean runLoop1 = new AtomicBoolean(true);

        // Realize the interface and assign it to the following code:
        NeoInsertMarker telemetryLoop1 = ()-> {
            // Simply print the current time into this code!
            ElapsedTime t = new ElapsedTime();
            while(t.milliseconds()/1000.0 < 5 && runLoop1.get()){
                telemetry.addData("Time in loop 1 ",t.milliseconds()/1000.0);
                telemetry.update();
            }
            telemetry.addLine("Finished loop 1 ");
            telemetry.update();
            runLoop1.set(false);
        };

        // Assign variable to not run the risk of repeating the marker code
        AtomicBoolean runLoop2 = new AtomicBoolean(true);

        NeoInsertMarker telemetryLoop2 = ()-> {
            // Simply print the current time into this code!
            ElapsedTime t = new ElapsedTime();
            while(t.milliseconds()/1000.0 < 5 && runLoop2.get()){
                telemetry.addData("Time in loop 2 ",t.milliseconds()/1000.0);
                telemetry.update();
            }
            telemetry.addLine("Finished loop 2 ");
            telemetry.update();
            runLoop2.set(false);
        };
        // Assign these realized interfaces to the LinearOpMode
        setMarkerList(new NeoMarkerList(telemetryLoop1, 0.1, telemetryLoop2, 1));


        initialize();

        executeAuto();
        Timer.sleep(10);
        stopMarkers();

        telemetry.addLine("Done.");
        telemetry.update();

        // Time to read the telemetry output
        Timer.sleep(5);

        // Create the markerList (inherited variable)
//        markerList = new NeoMarkerList(telemetryLoop1, 0, telemetryLoop2, 3);
//
//
////         Create the path sequence
//        PathSequence sequenceToFollow = new PathSequence(Drivetrain.DIFFY,
//                sequenceInstructions,
//                leftFront, leftBack, rightFront, rightBack,
//                wheelR);
//
//        // Add the path sequence to the robot
//        setPathSequence(sequenceToFollow);

        // Set the robot to initialize mode
//        initialize();
//
//        // Run the autonomous
//        executeAuto();
//        Timer.sleep(10);
    }
}
