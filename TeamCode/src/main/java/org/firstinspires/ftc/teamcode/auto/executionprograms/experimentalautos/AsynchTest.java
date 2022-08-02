package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.Line;
import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.MarkerList;
import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.Path;
import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.Robot;
import org.firstinspires.ftc.teamcode.auto.coyotesupport.broadsupport.Turn;

@Autonomous(name="AsynchTest")
public class AsynchTest extends Robot {

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        Path line = new Line(0.5, 0.2);
        Path turn = new Turn(-90, 0.2);

        addPath(line);
        addPath(turn);

        // Realize the interface and assign it to the following code:
        InsertMarker telemetryLoop1 = ()-> {
            // Simply print the current time into this code!
            ElapsedTime t = new ElapsedTime();
            while(t.milliseconds()/1000.0 < 3 ){
                telemetry.addData("Time in loop 1 ",t.milliseconds()/1000.0);
                telemetry.update();
            }

        };
        InsertMarker telemetryLoop2 = ()-> {
            // Simply print the current time into this code!
            ElapsedTime t = new ElapsedTime();
            while(t.milliseconds()/1000.0 < 3){
                telemetry.addData("Time in loop 2 ",t.milliseconds()/1000.0);
                telemetry.update();
            }
        };

        // Execute the markers at the corresponding times
        markerList = new MarkerList(telemetryLoop1, 0, telemetryLoop2, 1);

        initialize();

        // run the auto and any markers
        executeAuto();
    }
}
