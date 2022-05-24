package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.outoftheboxrobotics.neutrinoi2c.Rev2mDistanceSensor.AsyncRev2MSensor;


@Autonomous(name="PhotonDistance")
public class PhotonDistance extends LinearOpMode {
    Rev2mDistanceSensor sensor;
    AsyncRev2MSensor asyncSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(Rev2mDistanceSensor.class, "Distance");
        asyncSensor = new AsyncRev2MSensor(sensor);
        asyncSensor.setSensorAccuracyMode(AsyncRev2MSensor.AccuracyMode.MODE_HIGH_ACCURACY);

        while(!opModeIsActive()) {
            telemetry.addData("Distance", asyncSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
