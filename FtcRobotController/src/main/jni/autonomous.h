#ifndef AUTONOMOUS
#define AUTONOMOUS

#include "arm.h"
#include "drive.h"

void autonomousUpdate()
{
    updateRobot();
    customAutonomousUpdate();
}

//this doesn't compile right now
#if 0
//TODO: go 0-100 for vis instead of 0-1, for clarity
void driveDistIn(float dist, float vIs)
{
    if (dist < 0)
    {
        dist = fabs(dist);
        vIs = -vIs;
    }

    bool doInit = true;
    int right_enc_net = 0;
    int left_enc_net = 0;
    int right_prev = 0;
    int left_prev = 0;
    while (right_enc_net < dist * encoderticks_per_inch ||
           left_enc_net < dist * encoderticks_per_inch)
    {
        if (doInit)
        {
            autonomousUpdate(env, self); //Might not need this?
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;

        if (right_enc_net < dist * encoderticks_per_inch)
        {
            right_drive = vIs;
        }
        if (left_encs_net < dist * encoderticks_per_inch)
        {
            left_drive = vIs;
        }

        autonomousUpdate();
    }
}

void driveDistCm(float dist, float vIs)
{
    if (dist < 0)
    {
        dist = abs(dist);
        vIs = -vIs;
    }

    bool doInit = true;
    int right_enc_net = 0;
    int left_enc_net = 0;
    int right_prev = 0;
    int left_prev = 0;
    while (right_enc_net < dist * encoderticks_per_cm || left_enc_net < dist * encoderticks_per_cm)
    {
        if (doInit)
        {
            autonomousUpdate(); //Might not need this?
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;

        if (right_enc_net < dist * encoderticks_per_cm)
        {
            right_drive = vIs;
        }
        if (left_enc_net < dist * encoderticks_per_cm)
        {
            left_drive = vIs;
        }

        autonomousUpdate();
    }
}

#define side_slowing_constant 5
void driveOnCourseIn(float dist, float vIs,
                     float target_heading) //Assuming we start facing 180 degrees (intake side)
{
    if (dist < 0)
    {
        dist = abs(dist);
        vIs = -vIs;
    }

    bool doInit = true;
    int right_enc_net = 0;
    int left_enc_net = 0;
    int right_prev = 0;
    int left_prev = 0;
    while (right_enc_net < dist * encoderticks_per_inch ||
           left_enc_net < dist * encoderticks_per_inch)
    {
        if (doInit)
        {
            autonomousUpdate(); //Might not need this?
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;
        if (tolerantEquals(heading, target_heading, acceptableAngleError))
        {
            //TODO: Recovery code that makes sure we don't gain ticks by getting hit
            if (right_enc_net < dist * encoderticks_per_inch)
            {
                right_drive = vIs;
            }
            if (left_enc_net < dist * encoderticks_per_inch)
            {
                left_drive = vIs;
            }
        }
        else if(isAngleGreater(heading, target_heading)) //TODO: make it so it ramps up if we're running at less than 100
        {
            left_drive = vIs;
            right_drive -= side_slowing_constant;
        }
        else
        {
            left_drive -= side_slowing_constant;
            right_drive = vIs;
        }
        autonomousUpdate();
    }
}

void driveOnCourseCm(float dist, float vIs,
                     float target_heading) //Assuming we start facing 180 degrees (intake side)
{
    if (dist < 0)
    {
        dist = abs(dist);
        vIs = -vIs;
    }

    bool doInit = true;
    int right_enc_net = 0;
    int left_enc_net = 0;
    int right_prev = 0;
    int left_prev = 0;
    while (right_enc_net < dist * encoderticks_per_cm ||
           left_enc_net < dist * encoderticks_per_cm)
    {
        if (doInit)
        {
            autonomousUpdate(); //Might not need this?
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;
        if (tolerantEquals(heading, target_heading, acceptableAngleError))
        {
            //TODO: Recovery code that makes sure we don't gain ticks by getting hit
            if (right_enc_net < dist * encoderticks_per_cm)
            {
                right_drive = vIs;
            }
            if (left_enc_net < dist * encoderticks_per_cm)
            {
                left_drive = vIs;
            }
        }
        else if(isAngleGreater(heading, target_heading)) //TODO: make it so it ramps up if we're running at less than 100
        {
            left_drive = vIs;
            right_drive -= side_slowing_constant;
        }
        else
        {
            left_drive -= side_slowing_constant;
            right_drive = vIs;
        }
        autonomousUpdate();
    }
}
#endif

//TODO: predict when to start decelerating (drive feedforward)
void turnRelDeg(float & left_motor, float & right_motor, float angle, float vIs, float * heading)
{
    float target_angle = *heading+angle;
    while(!tolerantEquals(*heading, target_angle, acceptableAngleError))
    {
        #if 1 //turn with constant speed
        if(isAngleGreaterDeg(*heading, target_angle))
        {
            left_motor = vIs;
            right_motor = -vIs;
        }
        else
        {
            left_motor = -vIs;
            right_motor = vIs;
        }
        #else //turn with proportional control
        float angle_off_rad = target_angle-*heading;
        angle_off_rad = signedCanonicalizeAngleDeg(angle_off)*pi/180;

        left_motor = -vIs*angle_off;
        right_motor = vIs*angle_off;
        #endif
        autonomousUpdate();
    }
}

#endif
