package com.qualcomm.ftcrobotcontroller.opmodes;

public class IK_solver
{
    public static final float forearm_len = 13.5f;
    public static final float upperarm_len = 15.0f;
    
    public static final float elbow_radius = 1.0f;
    public static final float winch_radius = 1.0f;
    
    //public static final float elbow_0 = 0.0f; //the winch rotation where the winch length is 0
    //public static final float shoulder_0 = -(float)Math.PI/2.0f; //the shoulder rotation where the shoulder is pointing in the +x axis
    public static final float shoulder_min = 0;
    public static final float shoulder_max = 2.0f*(float)Math.PI/3.0f;
    
//derived constants:
    public static final float elbow_1 = //the winch rotation where the arm switches modes
        ((float)Math.sqrt(sq(upperarm_len)-sq(elbow_radius))/winch_radius);
    public static final float elbow_2 = //the winch rotation where the arm is completely closed (toward the spring side)
        ((float)(2.0f*((float)Math.PI)-Math.acos(elbow_radius/upperarm_len)-Math.acos(elbow_radius/forearm_len))*elbow_radius/winch_radius);
    
    static float sq(float a)
    {
        return a*a;
    }
    
    static float invSqrt(float a)
    {
        int ai = Float.floatToRawIntBits(a);
        int isqrti;
        float isqrt;
        isqrti = 0x5f375a86 - (ai >> 1); //magic
        isqrt = Float.intBitsToFloat(isqrti);
        isqrt *= (1.5f - (a*0.5f*isqrt*isqrt));
        isqrt *= (1.5f - (a*0.5f*isqrt*isqrt)); //second itteration, uncomment if greater accuracy is needed
        return isqrt;
    }
    
    //NOTE: these vector functions are 2d only
    static void scale(float[] v, float s)
    {
        v[0] *= s;
        v[1] *= s;
    }
    
    static void normalizeScale(float[] v, float s)
    {
        float normsq = v[0]*v[0]+v[1]*v[1];
        scale(v, invSqrt(normsq)*s);
    }

    /*
      inputs: hand[0] and hand[1] are the cordinates of the wanted hand position in inches, relative to the robot
      hand will be clamped if it is outside of the range of motion of the arm
    */
    public static float[] getArmTargets(float[] hand)
    {
        float[] arm_targets = new float[2]; //[0] -> shoulder, [1] -> elbow
        
        float dist = (float) Math.sqrt(sq(hand[0])+sq(hand[1]));
        //clamp hand motion
        if(dist > forearm_len+upperarm_len) normalizeScale(hand, forearm_len+upperarm_len);
        if(dist < forearm_len-upperarm_len) normalizeScale(hand, forearm_len-upperarm_len);
        //TODO: clamp when the arm will hit the frame
        
        float shoulder_offset = (float) Math.acos((sq(upperarm_len)+sq(dist)-sq(forearm_len))/(2.0f*dist*upperarm_len));
        //from law of cosines, forearm_len^2 = upperarm_len^2 + dist^2 - 2*dist*upperarm_len*cos(shoulder_offset)
        arm_targets[0] = (float) Math.atan2(hand[1], hand[0]);
        
        float shoulder_target = arm_targets[0]+shoulder_offset;
        if(true)//shoulder_target < shoulder_max)
        { //winch case
            arm_targets[0] = shoulder_target;
        }
        else
        { //pulley case
            arm_targets[0] -= shoulder_offset;
        }
        //if(arm_targets[0] < 0.0) arm_targets[0] += (float) Math.PI*2.0f;
        float[] elbow_pos = new float[]{upperarm_len*(float) Math.cos(arm_targets[0]), upperarm_len*(float) Math.sin(arm_targets[0])};
        arm_targets[1] = (float) Math.atan2(hand[1]-elbow_pos[1], hand[0]-elbow_pos[0])+(float)Math.PI-arm_targets[0];
        if(arm_targets[1] < 0.0) arm_targets[1] += (float) Math.PI*2.0f;
        /*
          outputs: arm_targets[1] and arm_targets[0] are the rotations of
          the elbow(from the potentiometer) and shoulder outputs in radians, respectively
        */
        return arm_targets;
    }
    
    /*
      inputs: hand[0] and hand[1] are the polar cordinates of the wanted hand position in inches and radians, relative to the robot
      hand will be clamped if it is outside of the range of motion of the arm
    */
    public static float[] getArmTargetsPolar(float[] hand)
    {
        float[] arm_targets = new float[2]; //[0] -> shoulder, [1] -> elbow
        
        float dist = hand[0];
        //clamp hand motion
        if(dist > forearm_len+upperarm_len) normalizeScale(hand, forearm_len+upperarm_len);
        if(dist < forearm_len-upperarm_len) normalizeScale(hand, forearm_len-upperarm_len);
        //TODO: clamp when the arm will hit the frame
        
        float shoulder_offset = (float) Math.acos((sq(upperarm_len)+sq(dist)-sq(forearm_len))/(2.0f*dist*upperarm_len));
        //from law of cosines, forearm_len^2 = upperarm_len^2 + dist^2 - 2*dist*upperarm_len*cos(shoulder_offset)
        
        float shoulder_target = hand[1]+shoulder_offset;
        if(true)//shoulder_target < shoulder_max)
        { //winch case
            arm_targets[0] = hand[1]+shoulder_offset;
        }
        else
        { //pulley case
            arm_targets[0] = hand[1]-shoulder_offset;
        }
        //if(arm_targets[0] < 0.0) arm_targets[0] += (float) Math.PI*2.0f;
        arm_targets[1] = (float) Math.acos((sq(upperarm_len)+sq(forearm_len)-sq(dist))/(2.0f*upperarm_len*forearm_len));
        /*
          outputs: arm_targets[1] and arm_targets[0] are the rotations of
          the elbow(from the potentiometer) and shoulder outputs in radians, respectively
        */
        return arm_targets;
    }
}
