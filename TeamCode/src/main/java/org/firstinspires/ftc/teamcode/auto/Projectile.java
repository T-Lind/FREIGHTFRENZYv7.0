package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;

@Autonomous(name="Projectile")
public class Projectile extends LinearOpMode {
    private DcMotorEx shooter;

    @Override
    public void runOpMode() throws InterruptedException {
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("S");

        waitForStart();
        while(opModeIsActive())
            shooter.setVelocity(6, AngleUnit.RADIANS);
    }
}
