package org.firstinspires.ftc.teamcode.TeleOPs.support;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.support.Toggle;

abstract public class TeleOpContainer extends OpMode {
    /**
     * Drivetrain object from FTCLib
     */
    protected MecanumDrive drive;

    /**
     * Streamlined IMU object from FTCLib
     */
    protected RevIMU imu;

    /**
     * Gamepad object from FTCLib
     */
    protected GamepadEx driverOp;

    /**
     * Stores the state of the field centric variable locally
     */
    private boolean FIELD_CENTRIC = false;

    /**
     * Toggle object to store whether or not the drivetrain is in field centric mode
     */
    private Toggle fieldCentric;

    /**
     * Timer to keep track of the current time into the match
     */
    private Timing.Timer matchTimer;

    /**
     * Status variable to signal if the timer is started
     */
    private boolean matchTimerStarted;

    /**
     * Constant to multiply velocities by to improve motion
     */
    private static final double VEL_MULTIPLIER = 0.80;

    /**
     * Method to get the field centric state variable for displaying in telemetry, etc.
     * @return If the drivetrain is in field centric mode or not
     */
    protected boolean getFieldCentric() {
        return FIELD_CENTRIC;
    }

    /**
     * Method to get the remaining time used in the match for displaying in telemetry, etc.
     * @return The time remaining in the match
     */
    protected long getRemainingMatchTime(){
        return matchTimer.remainingTime();
    }

    /**
     * Initialize the motors, zero power behavior, drivetrain object, imu, and whatever other
     * hardware you want to initialize every time
     */
    @Override
    public void init() {

        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER
        Motor frontLeft = new Motor(hardwareMap, "FL", Motor.GoBILDA.RPM_312);
        Motor frontRight = new Motor(hardwareMap, "FR", Motor.GoBILDA.RPM_312);
        Motor backLeft = new Motor(hardwareMap, "BL", Motor.GoBILDA.RPM_312);
        Motor backRight = new Motor(hardwareMap, "BR", Motor.GoBILDA.RPM_312);

        // Sets zero power behavior to braking
        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);


        drive = new MecanumDrive(
                frontLeft,
                frontRight,
                backLeft,
                backRight
        );

        // This is the built-in IMU in the REV hub.
        // We're initializing it by its default parameters
        // and name in the config ('imu'). The orientation
        // of the hub is important. Below is a model
        // of the REV Hub and the orientation axes for the IMU.
        //
        //                           | Z axis
        //                           |
        //     (Motor Port Side)     |   / X axis
        //                       ____|__/____
        //          Y axis     / *   | /    /|   (IO Side)
        //          _________ /______|/    //      I2C
        //                   /___________ //     Digital
        //                  |____________|/      Analog
        //
        //                 (Servo Port Side)
        //
        // (unapologetically stolen from the road-runner-quickstart)

        // Create the IMU object and initialize
        imu = new RevIMU(hardwareMap);
        imu.init();

        // Create the extended gamepad object
        driverOp = new GamepadEx(gamepad1);

        // Create a new toggle object for field-robot centric switching mid match
        fieldCentric = new Toggle(true);

        // Create the new timer object to keep track of match time
        matchTimer = new Timing.Timer(120);
        matchTimerStarted = false;

        // Initialize the mechanisms of the robot
        initSpecificMechanisms();
    }

    /**
     * Method of teleop code to loop over every time
     */
    @Override
    public void loop(){
        // Statement to only run once at the beginning of the loop
        if(!matchTimerStarted) {
            matchTimer.start();
            matchTimerStarted = true;
        }


        // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
        // These are related to the left stick x value, left stick y value, and
        // right stick x value respectively. These values are passed in to represent the
        // strafing speed, the forward speed, and the turning speed of the robot frame
        // respectively from [-1, 1].

        if (!FIELD_CENTRIC) {

            // For a robot centric model, the input of (0,1,0) for (leftX, leftY, rightX)
            // will move the robot in the direction of its current heading. Every movement
            // is relative to the frame of the robot itself.
            //
            //                 (0,1,0)
            //                   /
            //                  /
            //           ______/_____
            //          /           /
            //         /           /
            //        /___________/
            //           ____________
            //          /  (0,0,1)  /
            //         /     ↻     /
            //        /___________/

            // optional fourth parameter for squared inputs
            drive.driveRobotCentric(
                    convertSpeed(driverOp.getLeftX()),
                    convertSpeed(driverOp.getLeftY()),
                    convertSpeed(driverOp.getRightX()),
                    false
            );
        } else {

            // Below is a model for how field centric will drive when given the inputs
            // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
            // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
            // regardless of the heading.
            //
            //                   heading
            //                     /
            //            (0,1,0) /
            //               |   /
            //               |  /
            //            ___|_/_____
            //          /           /
            //         /           / ---------- (1,0,0)
            //        /__________ /

            // optional fifth parameter for squared inputs
            drive.driveFieldCentric(
                    convertSpeed(driverOp.getLeftX()),
                    convertSpeed(driverOp.getLeftY()),
                    convertSpeed(driverOp.getRightX()),
                    imu.getRotation2d().getDegrees(),   // gyro value passed in here must be in degrees
                    false
            );
        }
        // Update what type of driving type is used
        updateDriveType();

        // Alert the driver to the amount of time left through a voice command.
        alertToTiming();

        // Update the mechanisms
        updateMechanisms();
    }

    /**
     * Check to see whether or not to change the driving type
     */
    private void updateDriveType(){
        // Use the left bumper to switch whether or not the drive type is field centric
        fieldCentric.updateLeadingEdge(driverOp.getButton(GamepadKeys.Button.LEFT_BUMPER));
        FIELD_CENTRIC = fieldCentric.getToggleState();
    }

    /**
     * Alert the driver at different time intervals. Note that it takes a while to say the words
     * so they're started speaking ahead of where they should be.
     */
    private void alertToTiming() {
        if(matchTimer.elapsedTime() == 30)
            telemetry.speak("30 seconds into tele-op");
        else if(matchTimer.remainingTime() == 45)
            telemetry.speak("15 seconds until end game");
        else if(matchTimer.remainingTime() == 5)
            telemetry.speak("three");
        else if(matchTimer.remainingTime() == 3)
            telemetry.speak("two");
        else if(matchTimer.remainingTime() == 1)
            telemetry.speak("one");
    }

    private double convertSpeed(double input) {
        return VEL_MULTIPLIER * Math.pow(input, 3)+input*(1-VEL_MULTIPLIER);
    }

    /**
     * Abstract method to initialize specific mechanisms you want for a teleop
     */
    abstract protected void initSpecificMechanisms();

    /**
     * Abstract method to update any mechanisms that you're running
     */
    abstract protected void updateMechanisms();
}