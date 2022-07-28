package org.firstinspires.ftc.teamcode.TeleOPs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOPs.roadrunner.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.TeleOPs.roadrunner.advanced.TeleOpJustLocalizer;
import org.firstinspires.ftc.teamcode.TeleOPs.support.TeleOpContainer;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;


/**
 * Class to run a mecanum drivetrain
 * with basic TeleOp controls.
 */
@TeleOp(name="BasicOp")
public class BasicOp extends TeleOpContainer {
//    protected ServoEx servo;
    @Override
    protected void initSpecificMechanisms() {
//        servo = new SimpleServo(
//                hardwareMap, "s1", 0, 180
//        );

    }

    /**
     * Method from TeleOpContainer that must be used in order for the teleOp to work.
     * Either put setFieldCentric() or put setRobotCentric() at the start of the method.
     */
    @Override
    protected void updateMechanisms() {
        // Use the left bumper to switch whether or not the drive type is field centric
//        updateServo();

        telemetry.addData("Time left in match: ", getRemainingMatchTime());
        telemetry.addData("Field Centric Drive Type", getFieldCentric());

        telemetry.addData("x", getX());
        telemetry.addData("y", getY());
        telemetry.addData("heading", getAngle());
        telemetry.update();

    }


}
