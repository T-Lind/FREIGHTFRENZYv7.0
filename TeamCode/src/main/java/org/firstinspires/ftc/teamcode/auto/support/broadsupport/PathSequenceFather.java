package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import java.util.ArrayList;

public abstract class PathSequenceFather {
    // Common variables and objects among all path sequences
    protected ArrayList<NeoPath> trajectory;
    protected double wheelRadius;


    /**
     * Build each NeoPath in the trajectory.
     * @Precondition trajectory is not null
     * @Postcondition all paths have been successfully built
     */
    protected final void buildAll(){
        assert trajectory != null : "Trajectory in PathSequenceFather.buildAll() must not be null!";
        for(NeoPath path : trajectory)
            path.build();
    }
    /**
     * Build a specific trajectory
     * @param i the index of the NeoPath that you want to build.
     * @Precondition i is a valid index and trajectory is not null
     * @Postcondition the NeoPath at position i has been built.
     */
    protected final void build(int i){
        assert trajectory != null : "Trajectory in PathSequenceFather.build(...) must not be null!";
        assert i >= 0 : "The index i in PathSequenceFather.build(int i) must be greater than or equal to zero!";
        trajectory.get(i).build();
    }

    /**
     * @Precondition all paths have been build using the buildAll() method.
     * Note that this is not technically necessary but reduces lag time.
     * Sets the completed state of each path back to false to allow for path reusability
     * @Postcondition only the completed variable in each NeoPath in trajectory has been changed
     */

    protected final void resetPaths(){
        assert trajectory != null : "Trajectory in PathSequenceFather.resetPaths() must not be null!";
        for(NeoPath path : trajectory)
            path.setCompleted(false);
    }

    /**
     * Method to follow the path sequence
     */
    public abstract void follow();
}
