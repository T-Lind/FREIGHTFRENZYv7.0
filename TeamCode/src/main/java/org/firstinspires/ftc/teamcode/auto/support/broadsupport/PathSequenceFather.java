package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import java.util.ArrayList;

public abstract class PathSequenceFather {
    // Common variables and objects among all path sequences
    protected ArrayList<NeoPath> trajectory;
    protected double wheelRadius;


    /**
     * Build each NeoPath in the trajectory.
     */
    protected final void buildAll(){
        for(NeoPath p : trajectory)
            p.build();
    }
    /**
     * Build a specific trajectory
     * @param i the index of the NeoPath that you want to build.
     */
    protected final void build(int i){
        trajectory.get(i).build();
    }

    /**
     * @Precondition all paths have been build using the buildAll() method.
     * Note that this is not technically necessary but reduces lag time.
     * Sets the completed state of each path back to false to allow for path reusability
     * @Postcondition only the completed variable in each NeoPath in trajectory has been changed
     */

    protected final void resetPaths(){
        for(NeoPath p : trajectory)
            p.setCompleted(false);
    }

    /**
     * Method to follow the path sequence
     */
    public abstract void follow();
}
