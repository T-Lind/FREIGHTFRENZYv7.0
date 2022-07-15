package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOPs.support.TeleOpContainer;
import org.firstinspires.ftc.teamcode.auto.support.Toggle;


/**
 * Class to run a mecanum drivetrain
 * with basic TeleOp controls.
 */
@TeleOp(name="BasicOp")
public class BasicOp extends TeleOpContainer {
    Toggle fieldCentric = new Toggle(true);

    /**
     * Method from TeleOpContainer that must be used in order for the teleOp to work.
     * Either put setFieldCentric() or put setRobotCentric() at the start of the method.
     */
    @Override
    protected void updateMechanisms() {
        fieldCentric.updateFallingEdge(gamepad1.a);
        if(fieldCentric.getToggleState())
            setFieldCentric();
        else
            setRobotCentric();

        telemetry.addData("Hi!","");
        telemetry.update();
    }
}
