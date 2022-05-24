package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Turn;
import org.firstinspires.ftc.teamcode.auto.support.basicsupport.TwoWheelPathSequence;

import java.util.ArrayList;

@Autonomous(name="TestTurn")
public class TestTurn extends LinearOpMode {
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

        NeoPath turn = new Turn(90, 0.3683,0.5);

        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(turn);


        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048);
        sequence.buildAll();
        sequence.follow();


    }
}
