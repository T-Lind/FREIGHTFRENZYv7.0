package org.firstinspires.ftc.teamcode.auto.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.Line;
import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.SplinePath;
import org.firstinspires.ftc.teamcode.auto.support.basicsupport.TwoWheelPathSequence;

import java.util.ArrayList;

@Autonomous(name="Square")
public class Square extends LinearOpMode {
    private DcMotorEx left, right;

    @Override
    public void runOpMode() throws InterruptedException {
        left = (DcMotorEx) hardwareMap.dcMotor.get("L");
        right = (DcMotorEx) hardwareMap.dcMotor.get("R");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(!opModeIsActive()) {
            telemetry.addLine("Initialized.");
            telemetry.update();
        }

        double[] r = {.184,.184,.184,.184};
        double[] arcs = {-0.578/4,-0.578/4,-0.578/4,-0.578/4};
        NeoPath turn = new SplinePath(0.368,0.3,0.1,r,arcs);
        NeoPath turn2 = new SplinePath(0.368,0.3,0.1,r,arcs);
        NeoPath forward = new Line(0.4,0.5);
        NeoPath forward2 = new Line(0.4,0.5);


        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(forward);
        list.add(turn);
        list.add(forward2);

        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048);
        sequence.buildAll();
        sequence.follow();



    }
}
