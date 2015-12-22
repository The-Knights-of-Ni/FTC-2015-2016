package com.qualcomm.ftcrobotcontroller.opmodes;

/**
 * Created by Dev on 12/15/2015.
 */
public class Drive
{
    public static final float traction_reduction_constant = (float) 0.25;
    public static final float dist_to_treads = (float) 7.04;//Currently in inches, not sure if we have standard units
    public static final float encoder_velocity_error = (float) 1.0;//m/s currently

    private Drive(){}//Not used as an object.
    //TODO: Traction control
    private static float[] modifier = new float[2];//Should remember for every cycle, this is what the system is based around.
    public static float[] tractionControl(float rightVelocity, float leftVelocity, float omega, float v)
    {
        if(tolerantEquals(leftVelocity, rightVelocity, encoder_velocity_error) && leftVelocity == v)//Probably want some sort of tolerance function
        {
            modifier[0] = 1;
            modifier[1] = 1;
        }
        if(leftVelocity == rightVelocity && leftVelocity < v)//Case when we're just driving slower than normal, might need some sort of tolerance.
        {
            modifier[0] -= traction_reduction_constant;//Gradually slow the motors
            modifier[1] -= traction_reduction_constant;
        }
        if(leftVelocity > rightVelocity)
            modifier[0]--;
        if(rightVelocity > leftVelocity)
            modifier[1]--;
        return modifier;

    }
    private static boolean tolerantEquals(float val1, float val2, float absoluteTolerance)
    {
        if(val1 + absoluteTolerance > val2 && val2 > val1 - absoluteTolerance)//If va2 is within tolerance of val1
            return true;
        return false;
    }
    public float[] driveStraight(float heading, float target)//Call when stick is +- 1 y, need a state machine for this. (if stick forward, then set heading and run this)
    {
        if(target != heading)//Add tolerance
        {
            if(heading == -target)//If we spin out or something like that
            {
                modifier[0] *= 1;//put us in turn mode, until we're back at the heading.
                modifier[1] *= -1;
            }
            if(heading > target)//Bound at 180 or -180, I think that's the best way to wrap. TODO: this is pointless past a 90 degree offset, and probably even past 45
                modifier[1]--;//Slow right side until we get back to the heading, maybe make this so that it doesn't change driver speed so much
            if(heading < target)
                modifier[0]--;//Slow left side until back at the heading.
        }
        return modifier;
    }
    //TODO: tolerant equals
    //TODO: Drive straight
    //TODO: update function (hopefully not needed)
    //TODO: negative inertia
    //TODO: Quick turn (whatever that is)
    //TODO: field centric turn
    //TODO: field centric drive (maybe just make a turn to heading method)
    //TODO: Javadoc it
}
