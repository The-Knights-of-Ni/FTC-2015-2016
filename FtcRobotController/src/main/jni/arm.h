#ifndef ARM
#define ARM

#include "robotics.h"

#define potentiometer_range 333.33333333333333333333333333333333333

//TODO: Feed Forward
#define forearm_len 18.38
#define upperarm_len 16.56

#define elbow_radius 2.5 //Check this
#define winch_radius 1.0

//public static final float elbow_0 = 0.0f; //the winch rotation where the winch length is 0
//public static final float shoulder_0 = -(float)Math.PI/2.0f; //the shoulder rotation where the shoulder is pointing in the +x axis
#define shoulder_min  0
#define shoulder_max  2.0f*pi/3.0f

//derived constants:
#define elbow_1 sqrt(sq(upperarm_len)-sq(elbow_radius))/winch_radius); //the winch rotation where the arm switches modes
#define elbow_2 ((2.0f*((pi-acos(elbow_radius/upperarm_len)-acos(elbow_radius/forearm_len))*elbow_radius/winch_radius);


/*
inputs: hand[0] and hand[1] are the coordinates of the wanted hand position in inches, relative to the robot
hand will be clamped if it is outside of the range of motion of the arm
*/
v2f getArmTargetsRectangular(v2f hand, bool mode)
{
    v2f arm_targets; //0 -> shoulder, 1 -> elbow

    float dist = norm(hand);
    //clamp hand motion
    if (dist > forearm_len + upperarm_len)
        hand * ((0.9 * (forearm_len + upperarm_len)) * invSqrt(dist));
    if (dist < forearm_len - upperarm_len)
        hand * ((0.9 * (forearm_len - upperarm_len)) * invSqrt(dist));
    //TODO: clamp when the arm will hit the frame

    arm_targets[0] = atan2(hand[1], hand[0]);
    if (arm_targets[0] < 0.0f)
        arm_targets[0] += 2.0f * pi;

    float shoulder_offset = acos((sq(upperarm_len) + sq(dist) - sq(forearm_len)) / (2.0f * dist * upperarm_len));
    //from law of cosines, forearm_len^2 = upperarm_len^2 + dist^2 - 2*dist*upperarm_len*cos(shoulder_offset)

    arm_targets[1] = acos(
            (sq(upperarm_len) + sq(forearm_len) - sq(dist)) / (2.0f * upperarm_len * forearm_len));
    if (mode)//shoulder_target < shoulder_max)
    { //pulley case

        arm_targets[0] = arm_targets[0] - shoulder_offset;
        arm_targets[1] = 2.0f * pi - arm_targets[1];
    }
    else
    { //winch case
        arm_targets[0] = arm_targets[0] + shoulder_offset;
    }
    /*
      outputs: arm_target[1] and arm_targets[0] are the rotations of
      the elbow(from the potentiometer) and shoulder outputs in radians, respectively
    */
    return arm_targets;
}

/*
inputs: hand[0] and hand[1] are the polar coordinates of the wanted hand position in inches and radians, relative to the robot
 hand will be clamped if it is outside of the range of motion of the arm
*/
v2f getArmTargetsPolar(v2f hand, bool mode)
{
    v2f arm_targets; //[0] -> shoulder, [1] -> elbow

    //clamp hand motion
    if (hand[0] > forearm_len + upperarm_len)
        hand[0] = forearm_len + upperarm_len;
    if (hand[0] < abs(forearm_len - upperarm_len))
        hand[0] = abs(forearm_len - upperarm_len);
    float dist = hand[0];
    //TODO: clamp when the arm will hit the frame

    float shoulder_offset = acos(
            (sq(upperarm_len) + sq(dist) - sq(forearm_len)) / (2.0f * dist * upperarm_len));
    //from law of cosines, forearm_len^2 = upperarm_len^2 + dist^2 - 2*dist*upperarm_len*cos(shoulder_offset)

    arm_targets[1] = acos((sq(upperarm_len) + sq(forearm_len) - sq(dist)) / (2.0f * upperarm_len * forearm_len));
    if (mode)//shoulder_target < shoulder_max)
    { //pulley case
        arm_targets[0] = hand[1] - shoulder_offset;
        arm_targets[1] = 2.0f * pi- arm_targets[1];
    }
    else
    { //winch case
        arm_targets[0] = hand[1] + shoulder_offset;
    }

    /*
      outputs: arm_targets[1] and arm_targets[0] are the rotations of
      the elbow(from the potentiometer) and shoulder outputs in radians, respectively
    */
    return arm_targets;
}




#endif
