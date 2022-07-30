package org.firstinspires.ftc.teamcode.auto.roadrunner;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.RoadrunnerUnit;
import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.RoadrunnerWrapper;
import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.pathsequencewrapper.RoadrunnerWrapperPS;
import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.pathsequencewrapper.RobotCommonPS;
import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.pathsequencewrapper.SequenceWrapperPS;
import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.pathsequencewrapper.WrapperBuilderPS;

@Autonomous(name="RRSeparatePathWrapperTest")
public class RRSeparatePathWrapperTest extends LinearOpMode {

    /**
     * Override this method and place your code here.
     * <p>
     * Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.
     *
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException {
        RoadrunnerWrapper pathing = new RoadrunnerWrapper(hardwareMap, RoadrunnerUnit.CM);
        pathing.forward(10);
        pathing.lineToLinearHeading(50, 40, 45);
        pathing.back(20);
        pathing.addTemporalMarker(1, () -> {
            telemetry.addData("ran","");
            telemetry.update();
        });

        waitForStart();

        if(isStopRequested()) return;

        pathing.follow();
    }
}
