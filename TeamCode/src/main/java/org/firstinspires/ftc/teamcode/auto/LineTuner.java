package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.support.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.Line;
import org.firstinspires.ftc.teamcode.auto.support.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.SplinePath;
import org.firstinspires.ftc.teamcode.auto.support.Turn;
import org.firstinspires.ftc.teamcode.auto.support.TwoWheelPathSequence;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name="LineTuner")
public class LineTuner extends LinearOpMode {
    private DcMotorEx left, right;
    DistanceSensor distanceSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        left = (DcMotorEx) hardwareMap.dcMotor.get("L");
        right = (DcMotorEx) hardwareMap.dcMotor.get("R");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        while(!opModeIsActive()) {
            telemetry.addLine("Initialized.");
            telemetry.update();
        }


//        NeoPath line = new Line(0.5,.35);
//        NeoPath line2 = new Line(-0.5,.35);
//
//        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
//        list.add(line);
//        list.add(line2);
//        list.add(line);
//        list.add(line2);
//        list.add(line);
//        list.add(line2);
        NeoPath turnLeft = new Turn(90, 0.3683, 0.35);
        NeoPath turnRight = new Turn(-90, 0.3683, 0.35);
        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(turnLeft);
        list.add(turnRight);
        list.add(turnLeft);
        list.add(turnRight);
        list.add(turnLeft);
        list.add(turnRight);

        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048);

        sequence.buildAll();
        sequence.follow();


    }
}
