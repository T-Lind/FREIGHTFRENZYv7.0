package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.Line;
import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.SplinePath;
import org.firstinspires.ftc.teamcode.auto.support.Turn;
import org.firstinspires.ftc.teamcode.auto.support.TwoWheelPathSequence;

import java.util.ArrayList;

@Autonomous(name="TestDrivetrainFFAutoPath")
public class TestDrivetrainFFAutoPath extends LinearOpMode {
    private DcMotorEx left, right;

    @Override
    public void runOpMode() throws InterruptedException {
        left = (DcMotorEx) hardwareMap.dcMotor.get("L");
        right = (DcMotorEx) hardwareMap.dcMotor.get("R");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(!opModeIsActive()) {
            telemetry.addLine("Initialized.");
            telemetry.update();
        }

        // Duck Side
        NeoPath line = new Line(0.5,0.5);
        NeoPath turn = new Turn(-45, 0.3683,0.5);
        double[] r = {2,5,2};
        double[] arcs = {0.3,0.55,0.3};
        NeoPath splne = new SplinePath(0.368,0.5,0.2,r,arcs, true);
        double[] r2 = {0.35,0.5,0.2};
        double[] arcs2 = {0.4,0.3,0.25};
        NeoPath splne2 = new SplinePath(0.368,0.5,0.2,r2,arcs2);
        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(line);
        list.add(turn);
        list.add(line);
        list.add(splne);
        list.add(splne2);
        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048);
        sequence.buildAll();
        sequence.follow();


    }
}
