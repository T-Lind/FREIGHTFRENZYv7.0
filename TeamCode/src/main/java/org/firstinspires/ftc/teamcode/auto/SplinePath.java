package org.firstinspires.ftc.teamcode.auto;
/**
 * Translates a series of circle radii/arc lengths into motor power values for a 2 wheeled robot.
 * created by @author Tiernan Lindauer for FTC team 7797.
 * @license  Creative Commons
 * Last edited 4/18/22
 *
 */


public class SplinePath extends NeoPath {
    private double[] radii;
    private double[] arcLengths;
    private double velocity;
    private double trackWidth;
    private double accelerationTime;
    private double TpA;
    private double TpD;


    // track width is how far the wheels are apart, r is the radius of each of the turns, v is an ArrayList of static arrays of the velocities.
    /**
     * @Precondition each static array in v is 3 long, size of r and v is equal
     * @Convention positive arc length is CCW, negative is CW
     * **/
    public SplinePath(double tw, double v, double accTime, double[] r, double[] al){
        trackWidth = tw;
        radii = r;
        arcLengths = al;
        velocity = v;
        accelerationTime = accTime;
        TpA = 0;
        TpD = 0;

    }

    @Override
    public void build(){
        TpA = (3*Math.abs(arcLengths[0])-2*velocity*accelerationTime)/(3*velocity);
        TpD = (arcLengths[arcLengths.length-1]-((velocity*accelerationTime)/3))/velocity;
    }

    public int getArc(double t){
        double time = TpA;
        if(t < time)
            return 0;

        for(int i=1;i<radii.length-1;i++){
            time += velocity*=radii[i];
            if(t < time)
                return i;
        }
        time += TpD;
        if(t < time)
            return radii.length-1;
        this.setCompleted(true);
        return -1;
    }
    private double getVelocity(double t){
        double v = 0;
        if(getArc(t) == 0)
            v = velocity * Math.sqrt(t / accelerationTime);
        else if(getArc(t) < radii[radii.length-1])
            v = velocity;
        else
            if(getArc(t) != -1)
                v = velocity - velocity * Math.sqrt(t / accelerationTime);
        return v*(trackWidth/(2*radii[0]));
    }

    @Override
    public double getLeftVelocity(double t){
        if(getArc(t) == -1)
            return 0;
        double v = getVelocity(t)/(trackWidth/(2*radii[getArc(t)]));
        if(arcLengths[getArc(t)] < 0)
            return -1*(v+v*getVelocity(t));
        return -1*(v-v*getVelocity(t));
    }
    @Override
    public double getRightVelocity(double t){
        if(getArc(t) == -1)
            return 0;
        double v = getVelocity(t)/(trackWidth/(2*radii[getArc(t)]));
        if(arcLengths[getArc(t)] < 0)
            return (v-v*getVelocity(t));
        return (v+v*getVelocity(t));
    }
}
