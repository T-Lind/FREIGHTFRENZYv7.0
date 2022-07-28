package org.firstinspires.ftc.teamcode.TeleOPs;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOPs.support.TeleOpContainer;


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
        telemetry.update();
    }

//    private void updateServo() {
//        if(driverOp.getButton(GamepadKeys.Button.X)) {
//            servo.turnToAngle(0);
//        }
//
//        else if(driverOp.getButton(GamepadKeys.Button.Y)) {
//            servo.turnToAngle(90);
//        }
//
//        else if(driverOp.getButton(GamepadKeys.Button.B)) {
//            servo.turnToAngle(180);
//        }
//    }
}
