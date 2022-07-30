package org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.pathsequencewrapper;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

/**
 * Wrapper object for Roadrunner - uses the builder design pattern object WrapperBuilder
 */
public class SequenceWrapperPS {
    /**
     * Public object to be referenced later
     */
    public TrajectorySequenceBuilder trajectorySequenceBuilder;

    /**
     * Constructor to assign the trajectorySequenceBuilder
     * @param builder the wrapper for trajectorySequenceBuilder with the new features
     */
    public SequenceWrapperPS(WrapperBuilderPS builder) {
        trajectorySequenceBuilder = builder.trajectorySequenceBuilder;

    }

}
