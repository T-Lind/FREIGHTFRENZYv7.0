package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.Line;
import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
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

        NeoPath line = new Line(0.5,0.4);
        NeoPath turn = new Turn(45, 0.3683,0.5);

        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(line);
        list.add(turn);
        list.add(line);

        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048);
        sequence.buildAll();
        sequence.follow();


    }
}
