package org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

//TODO: JAVADOC THESE METHODS

public class WrapperBuilder {
    public TrajectorySequenceBuilder trajectorySequenceBuilder;
    private boolean metric;

    public WrapperBuilder(SampleMecanumDrive drive, Pose2d startPose) {
        trajectorySequenceBuilder = drive.trajectorySequenceBuilder(startPose);
        metric = false;
    }

    /**
     * Use *CENTIMETERS* INSTEAD OF INCHES
     * @return this object to use in a builder design pattern
     */
    public WrapperBuilder metric() {
        metric = true;
        return this;
    }

    public WrapperBuilder forward(double distance) {
        if(metric) {
            trajectorySequenceBuilder.forward(toInchesFromCentimeters(distance));
            return this;
        }
        trajectorySequenceBuilder.forward(distance);
        return this;
    }

    public WrapperBuilder back(double distance) {
        if(metric) {
            trajectorySequenceBuilder.back(toInchesFromCentimeters(distance));
            return this;
        }

        trajectorySequenceBuilder.back(distance);
        return this;
    }

    public WrapperBuilder strafeLeft(double distance) {
        if(metric) {
            trajectorySequenceBuilder.strafeLeft(toInchesFromCentimeters(distance));
            return this;
        }

        trajectorySequenceBuilder.strafeLeft(distance);
        return this;
    }

    public WrapperBuilder strafeRight(double distance) {
        if(metric) {
            trajectorySequenceBuilder.strafeRight(toInchesFromCentimeters(distance));
            return this;
        }

        trajectorySequenceBuilder.strafeRight(distance);
        return this;
    }

    public WrapperBuilder strafeTo(double x, double y) {
        if(metric) {
            trajectorySequenceBuilder.strafeTo(new Vector2d(toInchesFromCentimeters(x), toInchesFromCentimeters(y)));
            return this;
        }

        trajectorySequenceBuilder.strafeTo(new Vector2d(x, y));
        return this;
    }

    public WrapperBuilder lineTo(double x, double y) {
        if(metric) {
            trajectorySequenceBuilder.lineTo(new Vector2d(toInchesFromCentimeters(x), toInchesFromCentimeters(y)));
            return this;
        }

        trajectorySequenceBuilder.lineTo(new Vector2d(x, y));
        return this;
    }

    public WrapperBuilder lineToConstantHeading(double x, double y) {
        if(metric) {
            trajectorySequenceBuilder.lineToConstantHeading(new Vector2d(toInchesFromCentimeters(x), toInchesFromCentimeters(y)));
            return this;
        }

        trajectorySequenceBuilder.lineToConstantHeading(new Vector2d(x, y));
        return this;
    }

    public WrapperBuilder lineToLinearHeading(double x, double y, double endHeading) {
        if(metric) {
            trajectorySequenceBuilder.lineToLinearHeading(new Pose2d(toInchesFromCentimeters(x), toInchesFromCentimeters(y), Math.toRadians(endHeading)));
            return this;
        }

        trajectorySequenceBuilder.lineToLinearHeading(new Pose2d(x, y, Math.toRadians(endHeading)));
        return this;
    }

    public WrapperBuilder lineToSplineHeading(double x, double y, double endHeading) {
        if(metric) {
            trajectorySequenceBuilder.lineToSplineHeading(new Pose2d(toInchesFromCentimeters(x),
                    toInchesFromCentimeters(y), Math.toRadians(endHeading)));
            return this;
        }

        trajectorySequenceBuilder.lineToSplineHeading(new Pose2d(x, y, Math.toRadians(endHeading)));
        return this;
    }


    public WrapperBuilder splineTo(double x, double y, double endHeading) {
        if(metric) {
            trajectorySequenceBuilder.splineTo(new Vector2d(toInchesFromCentimeters(x),
                    toInchesFromCentimeters(y)), Math.toRadians(endHeading));
            return this;
        }

        trajectorySequenceBuilder.splineTo(new Vector2d(x, y), Math.toRadians(endHeading));
        return this;
    }

    public WrapperBuilder splineToConstantHeading(double x, double y, double endTangent) {
        if(metric) {
            trajectorySequenceBuilder.splineToConstantHeading(new Vector2d(toInchesFromCentimeters(x),
                    toInchesFromCentimeters(y)), Math.toRadians(endTangent));
            return this;
        }

        trajectorySequenceBuilder.splineToConstantHeading(new Vector2d(x, y), Math.toRadians(endTangent));
        return this;
    }

    public WrapperBuilder splineToLinearHeading(double x, double y, double endHeading, double endTangent) {
        if(metric) {
            trajectorySequenceBuilder.splineToLinearHeading(new Pose2d(toInchesFromCentimeters(x),
                    toInchesFromCentimeters(y), Math.toRadians(endHeading)), Math.toRadians(endTangent));
            return this;
        }

        trajectorySequenceBuilder.splineToLinearHeading(new Pose2d(x, y, Math.toRadians(endHeading)),
                Math.toRadians(endTangent));
        return this;
    }

    public WrapperBuilder splineToSplineHeading(double x, double y, double endHeading, double endTangent) {
        if(metric) {
            trajectorySequenceBuilder.splineToSplineHeading(new Pose2d(toInchesFromCentimeters(x),
                    toInchesFromCentimeters(y), Math.toRadians(endHeading)), Math.toRadians(endTangent));
            return this;
        }

        trajectorySequenceBuilder.splineToSplineHeading(new Pose2d(x, y, Math.toRadians(endHeading)),
                Math.toRadians(endTangent));
        return this;
    }

    public WrapperBuilder turn(double angle) {
        trajectorySequenceBuilder.turn(Math.toRadians(angle));
        return this;
    }

    public WrapperBuilder addTemporalMarker(double time, MarkerCallback callback) {
        trajectorySequenceBuilder.addTemporalMarker(0.0, time, callback);
        return this;
    }


    public WrapperBuilder build() {
        trajectorySequenceBuilder.build();
        return this;
    }

    private double toInchesFromCentimeters(double centimeters) {
        return 2.54*centimeters;
    }
}
