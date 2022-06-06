package org.firstinspires.ftc.teamcode.auto.executionprograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.InsertMarker;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Line;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.MarkerList;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.SplinePath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Turn;
import org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport.TwoWheelPathSequence;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name="TestDrivetrainFFAutoPath")
public class TestDrivetrainFFAutoPath extends LinearOpMode {
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

        AtomicBoolean b = new AtomicBoolean(false);
        InsertMarker m = (t)->{
            // Insert marker code here - use as a function of t.
            if(t > 3 && t < 10){
                if(distanceSensor.getDistance(DistanceUnit.MM) < 100) {
                    telemetry.addData("Distance sensor status", "close");
                    b.set(true);
                }

                else
                    telemetry.addData("Distance sensor status","far");
            }
            else if(t > 10 && b.get()){
                 telemetry.addData("Long time...","");
            }
            telemetry.update();
        };
//        InsertMarker m2 = (t2)->{
//            // Insert other marker code here - use as a function of t.
//            if(t2 > 3 && t2 < 3.1)
//                telemetry.addData("Marker two executing at three seconds","");
//            if(t2 > 4 && t2 < 5)
//                telemetry.addData("Marker two executing at five seconds","");
//            telemetry.update();
//        };

        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048, new MarkerList(m));

        sequence.buildAll();
        sequence.follow();


    }
}
