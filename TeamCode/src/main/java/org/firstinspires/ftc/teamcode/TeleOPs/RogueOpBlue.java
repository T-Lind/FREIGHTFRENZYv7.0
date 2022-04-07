package org.firstinspires.ftc.teamcode.TeleOPs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.EasyToggle;
import org.firstinspires.ftc.teamcode.PIDS.LiftPID;

@TeleOp(name="RogueOpBlue")
public class RogueOpBlue extends RogueOpRed {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, intake, lift, ducc;

    private Servo arm1, arm2, dep, fold, cap;
    private boolean direction, togglePrecision;
    Orientation angles;
    EasyToggle toggleUp = new EasyToggle("up", false, 1, false, false);
    EasyToggle toggleDown = new EasyToggle("down", false, 1, false, false);
    EasyToggle toggleIn = new EasyToggle("in", false, 1, false, false);
    EasyToggle toggleOut = new EasyToggle("out", false, 1, false, false);
    EasyToggle levelUp = new EasyToggle("lu", false, 1, false, false);
    EasyToggle levelDown = new EasyToggle("ld", false, 1, false, false);

    private double factor;
    //test
    boolean reverse;
    BNO055IMU imu;
    private LiftPID liftPID = new LiftPID(.0075, 0, .003);
    int top = 1000;
    int mid = 500;
    int shared = 250;
    int setPos = top;
    int liftError = 0;
    int liftTargetPos = 0;
    EasyToggle toggleA = new EasyToggle("a", false, 1, false, false);
    //declare a rev color sensor v3 called color
    private RevColorSensorV3 color;
    boolean intook = false;
    // initialize an elapsed time named test
    ElapsedTime test = new ElapsedTime();
    int element = 0;
    boolean liftTime = false;
    boolean checkTime = true;
    double capPos = .5;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        super.loop();

    }

    @Override
    public void duck() {
        if (gamepad1.left_bumper && gamepad1.a) {
            ducc.setPower(-1);
        } else if (gamepad1.left_bumper) {
            ducc.setPower(-.5);
        } else {
            ducc.setPower(0);
        }
    }
}

