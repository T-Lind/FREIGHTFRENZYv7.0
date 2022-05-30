package org.firstinspires.ftc.teamcode.auto.experimentalautos;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.Bot;
import org.firstinspires.ftc.teamcode.auto.support.Line;
import org.firstinspires.ftc.teamcode.auto.support.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.Turn;
import org.firstinspires.ftc.teamcode.auto.support.diffysupport.DiffyPathSequence;

import java.util.ArrayList;

@Autonomous(name="DiffyAutoTest")
public class DiffyAutoTest extends LinearOpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack;

    @Override
    public void runOpMode() throws InterruptedException {
//        Bot bot = new Bot();
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("BR");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        while(!opModeIsActive()) {
            telemetry.addLine("Initialized.");
            telemetry.update();
        }

        NeoPath line = new Line(0.75,1);
        ArrayList<NeoPath> list = new ArrayList<NeoPath>();
        list.add(line);

//        DiffyPathSequence sequence = new DiffyPathSequence(list, bot.LF(), bot.LB(), bot.RF(), bot.RB(), Bot.wheelR);
        DiffyPathSequence sequence = new DiffyPathSequence(list, leftFront, leftBack, rightFront, rightBack, Bot.wheelR);

        sequence.buildAll();
        sequence.follow();

    }
}
