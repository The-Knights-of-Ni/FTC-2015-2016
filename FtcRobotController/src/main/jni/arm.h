//
// Created by Dev on 1/4/2016.
//
#include "maths.h"

#ifndef ARM
#define ARM

//TODO: Feed Forward
#define forearm_len 13.5
#define upperarm_len 15.0

#define elbow_radius 1.0
#define winch_radius 1.0

//public static final float elbow_0 = 0.0f; //the winch rotation where the winch length is 0
//public static final float shoulder_0 = -(float)Math.PI/2.0f; //the shoulder rotation where the shoulder is pointing in the +x axis
#define shoulder_min  0
#define shoulder_max  2.0f*pi/3.0f

//derived constants:
#define elbow_1 sqrt(sq(upperarm_len)-sq(elbow_radius))/winch_radius); //the winch rotation where the arm switches modes
#define elbow_2 ((2.0f*((pi-acos(elbow_radius/upperarm_len)-acos(elbow_radius/forearm_len))*elbow_radius/winch_radius);
typedef struct v2f position;
float *getArmTargetsRectangular(struct v2f hand, bool mode)
{
    position arm_targets; //[0] -> shoulder, [1] -> elbow

        float dist = sqrt(sq(hand[0]) + sq(hand[1]));
        //clamp hand motion
        if (dist > forearm_len + upperarm_len)
            normalizeScale(hand, 0.9f * (forearm_len + upperarm_len));
        if (dist < forearm_len - upperarm_len)
            normalizeScale(hand, 0.9f * (forearm_len - upperarm_len));
        //TODO: clamp when the arm will hit the frame

        arm_targets[0] = (float) Math.atan2(hand[1], hand[0]);
        if (arm_targets[0] < 0.0f) arm_targets[0] += 2.0f * (float) Math.PI;

        float shoulder_offset = (float) Math.acos(
                (sq(upperarm_len) + sq(dist) - sq(forearm_len)) / (2.0f * dist * upperarm_len));
        //from law of cosines, forearm_len^2 = upperarm_len^2 + dist^2 - 2*dist*upperarm_len*cos(shoulder_offset)

        arm_targets[1] = (float) Math.acos((sq(upperarm_len) + sq(forearm_len) - sq(dist)) /
                                           (2.0f * upperarm_len * forearm_len));
        if (mode)//shoulder_target < shoulder_max)
        { //pulley case

            arm_targets[0] = arm_targets[0] - shoulder_offset;
            arm_targets[1] = 2.0f * (float) Math.PI - arm_targets[1];
        }
        else
        { //winch case
            arm_targets[0] = arm_targets[0] + shoulder_offset;
        }
        /*
          outputs: arm_targets[1] and arm_targets[0] are the rotations of
          the elbow(from the potentiometer) and shoulder outputs in radians, respectively
        */
        return arm_targets;
    }


//PID Control: UNTESTED, may not be ported correctly (In case the built in doesn't work)
float k_p;
float k_i;
float k_d;
float k_d2;

float i = 0.0f;
float p_old;
float d = 0.0f;
float p2_old;
float d2 = 0.0f;

void PIDController(float K_P, float K_I, float K_D, float K_D2, float initial_val,
                   float initial_val2)
{
    k_p = K_P;
    k_i = K_I;
    k_d = K_D;
    k_d2 = K_D2;
    p_old = initial_val;
    p2_old = initial_val2;
}

float getControl(float p, float p2, float dt)
{
    d2 = lerp((p2 - p2_old) / dt,
              d2,
              (float) Math.exp(-20.0 * dt));
    p2_old = p2;

    if (i != i) i = 0.0f; //this will trigger if i is NaN
    i += p * dt; //might want to try different integrators
    d = lerp((p - p_old) / dt,
             d,
             (float) Math.exp(-20.0 * dt));
    p_old = p;
    return k_p * p + k_i * i;//+k_d*d+k_d2*d2;
}


#endif
