package org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.roadrunner.wrapper.pathsequencewrapper.WrapperBuilderPS;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.ArrayList;

// TODO: Javadoc this class
// TODO: fix error - temporal marker not working right and causing Exception & Error
public class RoadrunnerWrapper {
    private RoadrunnerUnit unit;

    private ArrayList<Trajectory> listOfTrajectories;

    private HardwareMap hardwareMap;

    private SampleMecanumDrive drivetrain;

    private Pose2d startPose;

    /**
     * Conversion factor to convert metric values to inches. <strong>SHOULD NEVER CHANGE!</strong>
     */
    private static final double METRIC_CONVERSION_FACTOR = 0.3937008;

    public RoadrunnerWrapper(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        drivetrain = new SampleMecanumDrive(this.hardwareMap);
        unit = RoadrunnerUnit.IN;
        startPose = new Pose2d(0, 0, 0);
        drivetrain.setPoseEstimate(startPose);
        listOfTrajectories = new ArrayList<>();
    }

    public RoadrunnerWrapper(HardwareMap hardwareMap, RoadrunnerUnit unit){
        this.hardwareMap = hardwareMap;
        drivetrain = new SampleMecanumDrive(this.hardwareMap);
        this.unit = unit;
        startPose = new Pose2d(0, 0, 0);
        drivetrain.setPoseEstimate(startPose);
        listOfTrajectories = new ArrayList<>();
    }

    public RoadrunnerWrapper(HardwareMap hardwareMap, double x, double y, double heading){
        this.hardwareMap = hardwareMap;
        drivetrain = new SampleMecanumDrive(this.hardwareMap);
        unit = RoadrunnerUnit.IN;
        startPose = new Pose2d(toInches(x), toInches(y), Math.toRadians(heading));
        drivetrain.setPoseEstimate(startPose);
        listOfTrajectories = new ArrayList<>();
    }

    public RoadrunnerWrapper(HardwareMap hardwareMap, RoadrunnerUnit unit,
                             double x, double y, double heading){
        this.hardwareMap = hardwareMap;
        drivetrain = new SampleMecanumDrive(this.hardwareMap);
        this.unit = unit;
        startPose = new Pose2d(toInches(x), toInches(y), Math.toRadians(heading));
        drivetrain.setPoseEstimate(startPose);
        listOfTrajectories = new ArrayList<>();
    }

    public void follow() {
        for(Trajectory trajectory : listOfTrajectories) {
            try {
                drivetrain.followTrajectory(trajectory);
            }
            catch(Error | Exception e) {
                return;
            }
        }
    }


    public void forward(double distance) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .forward(toInches(distance))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .forward(toInches(distance))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void back(double distance) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .back(toInches(distance))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .back(toInches(distance))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void strafeLeft(double distance) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .strafeLeft(toInches(distance))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .strafeLeft(toInches(distance))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void strafeRight(double distance) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .strafeRight(toInches(distance))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .strafeRight(toInches(distance))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void lineTo(double x, double y) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(toInches(x), toInches(y)))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .lineTo(new Vector2d(toInches(x), toInches(y)))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void lineToLinearHeading(double x, double y, double endHeading) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(toInches(x), toInches(y), Math.toRadians(endHeading)))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .lineToLinearHeading(new Pose2d(toInches(x), toInches(y), Math.toRadians(endHeading)))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void lineToSplineHeading(double x, double y, double endHeading) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .lineToSplineHeading(new Pose2d(toInches(x), toInches(y), Math.toRadians(endHeading)))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .lineToSplineHeading(new Pose2d(toInches(x), toInches(y), Math.toRadians(endHeading)))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void splineTo(double x, double y, double endHeading) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(toInches(x), toInches(y)), Math.toRadians(endHeading))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .splineTo(new Vector2d(toInches(x), toInches(y)), Math.toRadians(endHeading))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void splineToConstantHeading(double x, double y, double endHeading) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(toInches(x), toInches(y)), Math.toRadians(endHeading))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .splineToConstantHeading(new Vector2d(toInches(x), toInches(y)), Math.toRadians(endHeading))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void splineToLinearHeading(double x, double y, double endHeading, double endTangent) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .splineToLinearHeading(new Pose2d(toInches(x), toInches(y),
                            Math.toRadians(endTangent)), Math.toRadians(endHeading))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .splineToLinearHeading(new Pose2d(toInches(x), toInches(y),
                            Math.toRadians(endTangent)), Math.toRadians(endHeading))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    public void splineToSplineHeading(double x, double y, double endHeading, double endTangent) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .splineToSplineHeading(new Pose2d(toInches(x), toInches(y),
                            Math.toRadians(endTangent)), Math.toRadians(endHeading))
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .splineToSplineHeading(new Pose2d(toInches(x), toInches(y),
                            Math.toRadians(endTangent)), Math.toRadians(endHeading))
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    /**
     * Add a temporal marker for a given time into the whole path sequence
     * @param time the time to execute the temporal marker
     * @param callback the temporal marker
     */
    public void addTemporalMarker(double time, MarkerCallback callback) {
        Trajectory trajectory;
        if(listOfTrajectories.size() == 0){
            trajectory = drivetrain.trajectoryBuilder(startPose)
                    .addTemporalMarker(time, callback)
                    .build();
        }
        else{
            int index = listOfTrajectories.size()-1;
            trajectory = drivetrain.trajectoryBuilder(listOfTrajectories.get(index).end())
                    .addTemporalMarker(time, callback)
                    .build();
        }
        listOfTrajectories.add(trajectory);
    }

    /**
     * Convert a distance to inches based on the set unit
     * @param distance the input distances
     * @return the distance in inches
     */
    private double toInches(double distance) {
        if(unit == RoadrunnerUnit.IN)
            return distance;
        if(unit == RoadrunnerUnit.FT)
            return distance*12;
        if(unit == RoadrunnerUnit.CM)
            return distance*METRIC_CONVERSION_FACTOR;
        if(unit == RoadrunnerUnit.M)
            return distance*METRIC_CONVERSION_FACTOR*100;
        if(unit == RoadrunnerUnit.KM)
            return distance*METRIC_CONVERSION_FACTOR*1000;
        throw new RuntimeException("Incorrect distance unit specified into WrapperBuilder.toInches(...)!");
    }
}
