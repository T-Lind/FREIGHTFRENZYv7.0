package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import java.util.ArrayList;

public abstract class PathSequenceFather {
    // Common variables and objects among all path sequences
    protected ArrayList<NeoPath> trajectory;
    protected double wheelRadius;
    protected NeoMarkerList markerList;
    protected RunnableCollective runnableCollectiveMarkerList;

    /**
     * Build each NeoPath in the trajectory.
     */
    protected void buildAll(){
        for(NeoPath p : trajectory)
            p.build();
    }
    /**
     * Build a specific trajectory
     * @param i the index of the NeoPath that you want to build.
     */
    protected void build(int i){
        trajectory.get(i).build();
    }

    /**
     * @Precondition all paths have been build using the buildAll() method.
     * Note that this is not technically necessary but reduces lag time.
     * Sets the completed state of each path back to false to allow for path reusability
     * @Postcondition only the completed variable in each NeoPath in trajectory has been changed
     */

    public void resetPaths(){
        for(NeoPath p : trajectory)
            p.setCompleted(false);
    }

    /**
     * Method to follow the path sequence
     */
    public abstract void follow();
}
