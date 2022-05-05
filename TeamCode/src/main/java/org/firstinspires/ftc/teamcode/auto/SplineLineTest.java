package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.Line;
import org.firstinspires.ftc.teamcode.auto.support.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.SplinePath;
import org.firstinspires.ftc.teamcode.auto.support.TwoWheelPathSequence;

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
        };
        InsertMarker m2 = (t2)->{
            // Insert other marker code here - use as a function of t.
        };

        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048, new MarkerList(m,m2,new double[]{2,5}));
        sequence.buildAll();
        sequence.follow();


    }
}
