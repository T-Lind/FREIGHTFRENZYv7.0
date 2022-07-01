package org.firstinspires.ftc.teamcode.auto.support.broadsupport;

import org.firstinspires.ftc.teamcode.auto.support.enumerations.PathType;

public class RotatePods extends Path{
    /**
     * Method returning the type of path this specific path is
     *
     * @return the correct type of path.
     */
    @Override
    public PathType getType() {
        return PathType.PODTURN;
    }
}
