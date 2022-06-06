package org.firstinspires.ftc.teamcode.auto.executionprograms.experimentalautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.broadsupport.BotContainer;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.Line;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.NeoPath;
import org.firstinspires.ftc.teamcode.auto.support.broadsupport.PathSequence;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Drivetrain;

import java.util.ArrayList;

@Autonomous(name="PodTurnRaw")
public class BotContainerTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BotContainer bot = new BotContainer(this);

        NeoPath linePath = new Line(0.5, 0.5);
        ArrayList<NeoPath> pathArrayList = new ArrayList<NeoPath>();
        pathArrayList.add(linePath);

        PathSequence sequence = new PathSequence(
                Drivetrain.DIFFY,
                pathArrayList,
                bot.leftFront, bot.leftBack,
                bot.rightFront, bot.rightBack,
                1.25);

        bot.setPathSequence(sequence);

        while (!opModeIsActive()) {
            // In initialization

        }

        bot.followPaths();
    }
}
