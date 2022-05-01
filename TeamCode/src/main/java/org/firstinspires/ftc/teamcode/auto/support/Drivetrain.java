package org.firstinspires.ftc.teamcode.auto.support;

import com.qualcomm.robotcore.hardware.DcMotorEx;

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
