package org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public class SequenceWrapper {
    public TrajectorySequenceBuilder trajectorySequenceBuilder;
    public SequenceWrapper(WrapperBuilder builder) {
        trajectorySequenceBuilder = builder.trajectorySequenceBuilder;

    }

}
