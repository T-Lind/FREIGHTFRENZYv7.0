package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Line;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Path;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequence;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Robot;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Turn;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Drivetrain;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name="AsynchTest")
public class AsynchTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        // BotContainer is inherited, meaning that anything you put in BotContainer will be here as
        // well (as long as you make it protected), so is LinearOpMode (which BotContainer inherits)


        // Create the sequence of paths
        ArrayList<Path> sequenceInstructions = new ArrayList<>();

        Path line = new Line(0.5, 0.2);
        Path turn = new Turn(90, 0.6, 0.2);

        sequenceInstructions.add(line);
        sequenceInstructions.add(turn);

        //         Create the path sequence
        PathSequence sequenceToFollow = new PathSequence(Drivetrain.DIFFY,
                sequenceInstructions,
                leftFront, leftBack, rightFront, rightBack,
                wheelR);

        // Add the path sequence to the robot
        setPathSequence(sequenceToFollow);

        // Temporal Markers (Insert Markers) to execute during the path


        // Realize the interface and assign it to the following code:
        InsertMarker telemetryLoop1 = ()-> {
            // Simply print the current time into this code!
            ElapsedTime t = new ElapsedTime();
            while(t.milliseconds()/1000.0 < 3 ){
                telemetry.addData("Time in loop 1 ",t.milliseconds()/1000.0);
                telemetry.update();
            }
            telemetry.addLine("Finished loop 1 ");
            telemetry.update();
        };
        InsertMarker telemetryLoop2 = ()-> {
            // Simply print the current time into this code!
            ElapsedTime t = new ElapsedTime();
            while(t.milliseconds()/1000.0 < 3){
                telemetry.addData("Time in loop 2 ",t.milliseconds()/1000.0);
                telemetry.update();
            }
            telemetry.addLine("Finished loop 2 ");
            telemetry.update();
        };
        // Assign these realized interfaces to the LinearOpMode (protected variable inherited)
        markerList = new MarkerList(telemetryLoop1, 0, telemetryLoop2, 1);

        initialize();

        // run the auto and any markers
        executeAuto();
    }
}
