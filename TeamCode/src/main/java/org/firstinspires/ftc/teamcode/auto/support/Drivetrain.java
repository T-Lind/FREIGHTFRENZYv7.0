package org.firstinspires.ftc.teamcode.auto.support;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * A class not used as of 5/5/22
 */
public class Drivetrain {
    public DcMotorEx leftTop;
    public DcMotorEx leftBottom;
    public DcMotorEx rightTop;
    public DcMotorEx rightBottom;

    public Drivetrain(DcMotorEx lf, DcMotorEx lb, DcMotorEx rf, DcMotorEx rb){
        leftTop = lf;
        leftBottom = lb;
        rightTop = rb;
        rightBottom = rf;
    }
}
