//package org.firstinspires.ftc.teamcode.TeleOPs.freightfrenzy;
//
//import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor;
//
//@TeleOp(name="Async2mExampleOpmode")
//public class Async2mExampleOpmode extends OpMode {
//    Rev2mDistanceSensor sensor;
//    AsyncRev2MSensor asyncSensor;
//    @Override
//    public void init() {
//        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "Distance");
//        asyncSensor = new AsyncRev2MSensor(sensor);
//
//        telemetry.addLine("Press a for high accuracy, x for balanced accuracy, and b for high speed");
//    }
//
//    @Override
//    public void loop(){
//            telemetry.addData("Distance", asyncSensor.getDistance(DistanceUnit.CM));
//            telemetry.addData("Last Reading", asyncSensor.getLastMeasurementTimestamp());
//
//            if(gamepad1.a){
//                asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_ACCURACY);
//            }
//            if(gamepad1.x){
//                asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_BALANCED);
//            }
//            if(gamepad1.b){
//                asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_SPEED);
//            }
//
//            telemetry.update();
//        }
//}
