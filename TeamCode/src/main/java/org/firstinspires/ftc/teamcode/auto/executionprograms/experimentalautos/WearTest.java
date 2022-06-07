//package org.firstinspires.ftc.teamcode.auto;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.auto.support.InsertMarker;
//import org.firstinspires.ftc.teamcode.auto.support.Line;
//import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
//import org.firstinspires.ftc.teamcode.auto.support.SplinePath;
//import org.firstinspires.ftc.teamcode.auto.support.Turn;
//
//
//import java.util.ArrayList;
//import java.util.concurrent.atomic.AtomicBoolean;
//
//@Autonomous(name="WearTest")
//public class WearTest extends LinearOpMode {
//    private DcMotorEx left, right;
//    DistanceSensor distanceSensor;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        left = (DcMotorEx) hardwareMap.dcMotor.get("L");
//        right = (DcMotorEx) hardwareMap.dcMotor.get("R");
//        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance");
//
//        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        while(!opModeIsActive()) {
//            telemetry.addLine("Initialized.");
//            telemetry.update();
//        }
//
//        // Duck Side
//        NeoPath line = new Line(1000,2);
//        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
//        list.add(line);
//
//        TwoWheelPathSequence sequence = new TwoWheelPathSequence(list, left, right, 0.048);
//
//        sequence.buildAll();
//        sequence.follow();
//
//
//    }
//}
