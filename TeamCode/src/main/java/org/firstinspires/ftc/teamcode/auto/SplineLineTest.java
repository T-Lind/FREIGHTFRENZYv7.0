package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.Line;
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
//        NeoPath trajectory2 = new Line(0.75 ,0.6);
//        NeoPath trajectory3 = new Line(-0.5,0.2);
//        NeoPath trajectory4 = new Line(0.5,0.8);
//
//        double[] r2 = {0.6,0.6,2.5,2.5};
//        double[] arcs2 = {0.5,0.5,0.1,-0.1};
//
//        NeoPath trajectory5 = new SplinePath(0.368,0.4,0.3,r2,arcs2);

        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(trajectory1);
//        list.add(trajectory2);
//        list.add(trajectory3);
//        list.add(trajectory4);
//        list.add(trajectory5);

        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048);
        sequence.buildAll();
        sequence.follow();


    }
}
