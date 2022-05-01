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
            telemetry.addData("working", " pretty well");
            telemetry.update();
        }

        double[] r = {1,0.7,1.2,1,.184};
        double[] arcs = {0.4,0.2,-0.4,0.1,-0.578};

        NeoPath trajectory1 = new SplinePath(0.368,0.4,0.3,r,arcs);
        NeoPath trajectory2 = new Line(-0.5,0.6);
        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(trajectory1);
        list.add(trajectory2);

        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048);
        sequence.buildAll();
        sequence.follow();



    }
}
