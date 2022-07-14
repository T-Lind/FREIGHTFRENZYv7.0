package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TeleOPs.support.TeleopContainer;

/**
 * Field centric mecanum TeleOp code
 */
@Autonomous(name="FieldCentricMecanum")
public class FieldCentricMecanum extends TeleopContainer {
    /**
     * User defined loop method
     * This method will be called repeatedly in a loop while this op mode is running
     */
    @Override
    public void loop() {
        drive();

        telemetry.addData("IMU measurement",imu.getAngularOrientation().firstAngle);
    }


    /**
     * Code to drive the drivetrain.
     * Sets motor powers for the 4
     * drivetrain motors.
     */
    public void drive() {
        // Check to see if the gamepad is being pressed
        if (Math.abs(gamepad1.left_stick_y) > 0.1 || Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.right_stick_x) > 0.1) {
            // Calculate the motor powers based on stick angles
            double FLP = -(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            double FRP = -(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            double BLP = -(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
            double BRP = -(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);

            // Raise all motor powers to the power of 3 to enable better control for the driver
            FLP = Math.pow(FLP, 3);
            FRP = Math.pow(FRP, 3);
            BLP = Math.pow(BLP, 3);
            BRP = Math.pow(BRP, 3);

            // Calculate the maximum motor power at the moment
            double max = Math.max(Math.max(Math.abs(FLP), Math.abs(FRP)), Math.max(Math.abs(BLP), Math.abs(BRP)));

            /*
                If the maximum value is greater than 1, which is the greatest motor power that can
                be run at, then rescale the values back to a 0-1 range. It pretty much will always
                be greater than 1.
             */
            if (max > 1) {
                FLP /= max;
                FRP /= max;
                BLP /= max;
                BRP /= max;
            }

            // Precision mode ability - scale down motor power by the amount it's pressed
            float precisionModePercent = 1-gamepad1.left_trigger;
            if(precisionModePercent > PRECISION_MODE_MIN) {
                FLP *= precisionModePercent;
                FRP *= precisionModePercent;
                BLP *= precisionModePercent;
                BRP *= precisionModePercent;
            }
            else {
                FLP *= PRECISION_MODE_MIN;
                FRP *= PRECISION_MODE_MIN;
                BLP *= PRECISION_MODE_MIN;
                BRP *= PRECISION_MODE_MIN;
            }

            // Set the motor powers
            leftFront.setPower(FLP);
            rightFront.setPower(FRP);
            leftBack.setPower(BLP);
            rightBack.setPower(BRP);

            // Update telemetry
            telemetry.addData("Left trigger amount:", precisionModePercent);
        } else {
            // If the sticks are not being pressed then set the motor power values to zero
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }
    }
}
