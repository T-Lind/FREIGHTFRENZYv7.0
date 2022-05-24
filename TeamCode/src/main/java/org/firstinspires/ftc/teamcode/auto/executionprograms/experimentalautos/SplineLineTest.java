package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.SplinePath;
import org.firstinspires.ftc.teamcode.auto.support.basicsupport.TwoWheelPathSequence;

import java.util.ArrayList;

@Autonomous(name="SplineLineTest")
public class SplineLineTest extends LinearOpMode {
    private DcMotorEx left, right;

    @Override
    public void runOpMode() throws InterruptedException {
        left = (DcMotorEx) hardwareMap.dcMotor.get("L");
        right = (DcMotorEx) hardwareMap.dcMotor.get("R");

        while(!opModeIsActive()) {
            telemetry.addLine("Initialized.");
            telemetry.update();
        }
        double[] r = {0.6,0.6,0.6};
        double[] arcs = {-0.3,-0.3,-0.3};

        NeoPath trajectory1 = new SplinePath(0.368,0.4,0.3,r,arcs, true);

        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(trajectory1);

        InsertMarker m = (t)->{
            // Insert marker code here - use as a function of t.
            if(t > 2)
                telemetry.addLine("Marker one executing at two seconds");
            if(t > 5)
                telemetry.addLine("Marker one executing at five seconds");
            telemetry.update();
        };
        InsertMarker m2 = (t2)->{
            // Insert other marker code here - use as a function of t.
            if(t2 > 3.5)
                telemetry.addLine("Marker two executing at three point five seconds");
            if(t2 > 5)
                telemetry.addLine("Marker two executing at five seconds");
            telemetry.update();
        };

        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048, new MarkerList(m,m2));
        sequence.buildAll();
        sequence.follow();


    }
}
