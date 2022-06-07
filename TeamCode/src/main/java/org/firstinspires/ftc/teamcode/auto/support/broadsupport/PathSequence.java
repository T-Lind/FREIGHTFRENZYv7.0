package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport.FourWheelPathSequence;
import org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport.SixWheelPathSequence;
import org.firstinspires.ftc.teamcode.auto.support.basicdrivetrainsupport.TwoWheelPathSequence;
import org.firstinspires.ftc.teamcode.auto.support.diffysupport.DiffyPathSequence;
import org.firstinspires.ftc.teamcode.auto.support.enumerations.Drivetrain;

import java.util.ArrayList;

public class PathSequence {
    private PathSequenceFather sequence;
    private Drivetrain drivetrainType;

    // 2wd Path Sequences
    public PathSequence(Drivetrain drivetrainType, ArrayList<NeoPath> d, DcMotorEx left, DcMotorEx right, double wheelR){
        sequence = new TwoWheelPathSequence(d, left, right, wheelR);
    }
    public PathSequence(Drivetrain drivetrainType, ArrayList<NeoPath> d, DcMotorEx left, DcMotorEx right, double wheelR, NeoMarkerList markerList){
        sequence = new TwoWheelPathSequence(d, left, right, wheelR, markerList);
    }

    // 4wd/Diffy Path Sequences
    public PathSequence(Drivetrain drivetrainType, ArrayList<NeoPath> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx right1, DcMotorEx right2, double wheelR){
        if( drivetrainType == Drivetrain.FOURWD)
            sequence = new FourWheelPathSequence(paths, left1, left2, right1, right2, wheelR);
        else if( drivetrainType == Drivetrain.DIFFY)
            sequence = new DiffyPathSequence(paths, left1, left2, right1, right2, wheelR);
    }
    public PathSequence(Drivetrain drivetrainType, ArrayList<NeoPath> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx right1, DcMotorEx right2, double wheelR, NeoMarkerList markerList){
        if( drivetrainType == Drivetrain.FOURWD)
            sequence = new FourWheelPathSequence(paths, left1, left2, right1, right2, wheelR, markerList);
        else if( drivetrainType == Drivetrain.DIFFY)
            sequence = new DiffyPathSequence(paths, left1, left2, right1, right2, wheelR, markerList);
    }

    // 6wd Path Sequences
    public PathSequence(Drivetrain drivetrainType, ArrayList<NeoPath> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx left3, DcMotorEx right1, DcMotorEx right2, DcMotorEx right3, double wheelR){
        sequence = new SixWheelPathSequence(paths, left1, left2, left3, right1, right2, right3, wheelR);
    }
    public PathSequence(Drivetrain drivetrainType, ArrayList<NeoPath> paths, DcMotorEx left1, DcMotorEx left2, DcMotorEx left3, DcMotorEx right1, DcMotorEx right2, DcMotorEx right3, double wheelR, NeoMarkerList markerList){
        sequence = new SixWheelPathSequence(paths, left1, left2, left3, right1, right2, right3, wheelR, markerList);
    }

    public PathSequenceFather getPathSequence(){
        return sequence;
    }

    public void buildAll(){
        sequence.buildAll();
    }
    public void follow(){
        sequence.follow();
    }
}
