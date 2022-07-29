package org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class SequenceWrapper {
    private TrajectorySequence trajectorySequence;
    public TrajectorySequenceBuilder trajectorySequenceBuilder;
    private SampleMecanumDrive drive;
    public SequenceWrapper(WrapperBuilder builder) {
        trajectorySequenceBuilder = builder.trajectorySequenceBuilder;

    }

}
