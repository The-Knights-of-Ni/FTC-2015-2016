#ifndef AUTONOMOUS
#define AUTONOMOUS

#include "arm.h"
#include "drive.h"

void autonomousUpdate()
{
    customAutonomousUpdate();
    updateRobot();
    *heading /= 16.0;
}

void wait(float wait_time)
{
    float target_time = current_time + wait_time;
    while (current_time < target_time)
    {
        autonomousUpdate();
    }
}

#define side_slowing_constant 10

//this doesn't compile right now
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
    right_prev = *right_drive_encoder;
    left_prev = *left_drive_encoder;
#if 1
    while (right_enc_net < dist * encoderticks_per_inch)
    {
        *right_drive = vIs;
        *left_drive = vIs;
        autonomousUpdate();
        right_enc_net = *right_drive_encoder-right_prev;
    }
#else
    while (right_enc_net < dist * encoderticks_per_inch ||
           left_enc_net < dist * encoderticks_per_inch)
    {
        if (doInit)
        {
            autonomousUpdate(); //Might not need this?
            right_prev = *right_drive_encoder;
            left_prev = *left_drive_encoder;
            doInit = false;
        }
        right_enc_net = *right_drive_encoder - right_prev;
        left_enc_net = *left_drive_encoder - left_prev;
        
        if (right_enc_net < dist * encoderticks_per_inch)
        {
            *right_drive = vIs;
        }
        if (left_enc_net < dist * encoderticks_per_inch)
        {
            *left_drive = vIs;
        }
        
        autonomousUpdate();
    }
#endif
    *right_drive = 0;
    *left_drive = 0;
}

void driveDistCm(float dist, float vIs)
{
    driveDistIn(dist / 2.54, vIs);
}

void driveOnCourseIn(float dist, float vIs,
                     float target_heading) //Assuming we start facing 180 degrees (intake side)
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
    while (right_enc_net+left_enc_net < 2*dist * encoderticks_per_inch)
    {
        if (doInit)
        {
            autonomousUpdate(); //Might not need this?
            right_prev = *right_drive_encoder;
            left_prev = *left_drive_encoder;
            doInit = false;
        }
        right_enc_net = *right_drive_encoder - right_prev;
        left_enc_net = *left_drive_encoder - left_prev;
        if (tolerantEquals(*heading, target_heading, acceptableAngleError))
        {
            //TODO: Recovery code that makes sure we don't gain ticks by getting hit
#if 0
            //this might cause us to turn at the very end
            if (right_enc_net < dist * encoderticks_per_cm)
            {
                *right_drive = vIs;
            }
            if (left_enc_net < dist * encoderticks_per_cm)
            {
                *left_drive = vIs;
            }
#else
            *right_drive = vIs;
            *left_drive = vIs;
#endif
        }
        else if (isAngleGreaterDeg(*heading,
                                   target_heading)) //TODO: make it so it ramps up if we're running at less than 100
        {
            *right_drive = vIs;
            *left_drive = -vIs;//side_slowing_constant * dt;
        }
        else
        {
            *right_drive = -vIs;//side_slowing_constant * dt;
            *left_drive = vIs;
        }
        autonomousUpdate();
    }
    *left_drive = 0;
    *right_drive = 0;
}

void driveOnCourseCm(float dist, float vIs,
                     float target_heading) //Assuming we start facing 180 degrees (intake side)
{
    driveOnCourseIn(dist / 2.54, vIs, target_heading);
}

//TODO: predict when to start decelerating (drive feedforward)
void turnRelDeg(float angle, float vIs)
{
    float target_heading = *heading + angle;
    while (!tolerantEquals(*heading, target_heading, acceptableAngleError))
    {
#if 1 //turn with constant speed
        if (isAngleGreaterDeg(*heading, target_heading))
        {
            *left_drive = vIs;
            *right_drive = -vIs;
        }
        else
        {
            *left_drive = -vIs;
            *right_drive = vIs;
        }
#else //turn with proportional control
        float angle_off_rad = target_heading-*heading;
        angle_off_rad = signedCanonicalizeAngleDeg(angle_off)*pi/180;

        *left_drive = -vIs*angle_off;
        *right_drive = vIs*angle_off;
#endif
        autonomousUpdate();
    }
    *right_drive = 0;
    *left_drive = 0;
}

#endif
