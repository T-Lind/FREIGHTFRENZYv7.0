package org.firstinspires.ftc.teamcode.auto.bezierpathing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.bezierpathing.bezierconstruct.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.bezierpathing.bezierconstruct.Point;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name="BezierFollower")
public class BezierFollower extends LinearOpMode {
    public double max(double a, double b, double c, double d) {
        if(a > b && a > c && a > d)
            return a;
        if(b > a && b > c && b > d)
            return b;
        if(c > a && c > b && c > d)
            return c;
        return d;
    }

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
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        StandardTrackingWheelLocalizer myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap);
        myLocalizer.setPoseEstimate(new Pose2d(0,0));


        waitForStart();

        ArrayList<Point> points = new ArrayList<>();
        points.add(new Point(0,0));
        points.add(new Point(-10,15));
        points.add(new Point(6,-2));

        BezierCurve b = new BezierCurve(points);

        ElapsedTime timer = new ElapsedTime();

        PIDFController pidX = new PIDFController(0.7, 0.3, 0.3, 0.5);
        PIDFController pidY = new PIDFController(0.2, 0.3, 0.3, 0.5);
        PIDFController headingPID = new PIDFController(0.7, 0.5, 0.3, 0.4);

        while(timer.milliseconds() < 2000) {
            Point p = b.getCurrentPoint(timer.milliseconds()/2000.0);
            Pose2d myPose = myLocalizer.getPoseEstimate();

            double corX = pidX.calculate(p.x, myPose.getX());
            double corY = pidY.calculate(p.y, myPose.getY());
            double corHeading = headingPID.calculate(0, myPose.getHeading());

            double leftFront = -corY - corX + corHeading;
            double leftBack = -corY + corX + corHeading;
            double rightBack = -corY - corX + corHeading;
            double rightFront = -corY + corX + corHeading;

            double max = max(leftFront, leftBack, rightBack, rightFront);

            if(max > 1){
                leftFront/=max;
                rightFront/=max;
                leftBack/=max;
                rightBack/=max;
            }

            leftFront*=0.1;
            leftBack*=0.5;
            rightFront*=0.5;
            rightBack*=0.5;

            drive.setMotorPowers(leftFront, leftBack, rightBack, rightFront);
        }
    }
}
