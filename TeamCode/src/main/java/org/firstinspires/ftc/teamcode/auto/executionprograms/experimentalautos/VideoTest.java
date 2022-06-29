package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Line;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Path;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequence;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Robot;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Turn;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Drivetrain;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.DrivetrainSymmetry;

import java.util.ArrayList;

@Autonomous(name="VideoTest")
public class VideoTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        // Assign the hardware
        initializeHardware();

        // List of paths

        ArrayList<Path> sequenceOfInstructions = new ArrayList<>();

        Path line = new Line(0.5, 0.5, true);
        Path turn = new Turn(-90, 0.3, true);

        sequenceOfInstructions.add(line);
        sequenceOfInstructions.add(turn);

        // Path sequence

//        PathSequence sequenceToFollow = new PathSequence(
//                Drivetrain.DIFFY,
//                sequenceOfInstructions,
//                leftFront, leftBack, rightFront, rightBack,
//                wheelR
//        );
//
//        setPathSequence(sequenceToFollow);

        // Temporal markers (InsertMarkers) to execute during the path

        InsertMarker telemetryLoop1 = ()-> {
            telemetry.addData("This first marker is running!","");
            telemetry.update();
        };
        InsertMarker telemetryLoop2 = ()-> {
            telemetry.addData("This second marker is running!","");
            telemetry.update();
        };

        markerList = new MarkerList(telemetryLoop1, 3, telemetryLoop2, 5);


        // Initialize the program
        initialize();

        // Run the program
        executeAuto();
    }
}
