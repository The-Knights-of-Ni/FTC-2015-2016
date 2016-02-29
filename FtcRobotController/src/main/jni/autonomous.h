#ifndef AUTONOMOUS
#define AUTONOMOUS
#define BUTTON //This should make it so we don't have buttons we don't need in auto
#include "white_rabbit.h"

struct imu_state
{
    v3s orientation;
    v3f velocity;
};

imu_state * pimu_values;
//#define imu_heading (pimu_values->orientation.x)
float imu_heading = 0;
float imu_tilt = 0;
float imu_roll = 0;
float imu_heading_omega = 0;
float imu_tilt_omega = 0;
float imu_roll_omega = 0;
/* #define imu_tilt (pimu_values->orientation.y) */
/* #define imu_roll (pimu_values->orientation.z) */
#define imu_vel (pimu_values->velocity)

void (*customAutonomousUpdate)();

v3f imu_orientation_offsets;

void autonomousUpdate()
{
    dt = time-current_time;
    current_time = time;
    
    updateDriveSensors();
    updateArmSensors();
    customAutonomousUpdate();
    updateRobot();
    if(imu_heading_omega != imu_heading_omega) imu_heading_omega = 0;
    v3f current_orientation = {pimu_values->orientation.x, pimu_values->orientation.y, pimu_values->orientation.z};
    v3f imu_orientation = current_orientation - imu_orientation_offsets;
    lowpassFirstDerivativeUpdate(imu_orientation.x/-16.0, &imu_heading, &imu_heading_omega, 10);
    lowpassFirstDerivativeUpdate(imu_orientation.y/-16.0, &imu_tilt, &imu_tilt_omega, 10);
    lowpassFirstDerivativeUpdate(imu_orientation.z/-16.0, &imu_roll, &imu_roll_omega, 10);
}

void wait(float wait_time)
{
    float target_time = current_time + wait_time;
    while (current_time < target_time)
    {
        autonomousUpdate();
    }
}

void waitForEnd()
{
    for ever autonomousUpdate();
}

#define side_slowing_constant 10

#define default_max_acceleration 18

#define drive_dist_tolerance 0.5 //0.1 inch of acceptable error per drive
#define turning_deadband 10 //if there is more than this many degrees of error, the robot will stop driving and just turn

float * pdrive_time = 0;
float * pacceleration_time = 0;

//TODO: go 0-100 for vis instead of 0-1, for clarity //NOTE (Kyler): I like 0-1
void driveDistIn(float dist, float vIs, float max_acceleration = default_max_acceleration)
{
    if (dist < 0.0)
    {
        dist = fabs(dist);
        vIs = -vIs;
    }
    
    float start_drive_theta = avg_drive_theta;
    float current_dist = 0;
    
    while(fabs(dist-current_dist) > drive_dist_tolerance)
    {
        float drive_error = dist-current_dist;
        left_drive = sign(vIs)*drive_kp*drive_error;
        right_drive = sign(vIs)*drive_kp*drive_error;
        left_drive = clamp(left_drive, -fabs(vIs), fabs(vIs));
        right_drive = clamp(right_drive, -fabs(vIs), fabs(vIs));
        autonomousUpdate();
        current_dist = sign(vIs)*(avg_drive_theta-start_drive_theta)*sprocket_pitch_radius;
    }
    
    right_drive = 0;
    left_drive = 0;
}

inline void driveDistCm(float dist, float vIs, float max_acceleration = default_max_acceleration*cm)
{
    driveDistIn(dist/cm, vIs, max_acceleration/cm);
}

void driveOnCourseIn(float dist, float vIs,
                     float target_heading, //Assuming we start facing 180 degrees (intake side)
                     float max_acceleration = default_max_acceleration)
{
    if (dist < 0.0)
    {
        dist = fabs(dist);
        vIs = -vIs;
    }
    
    float start_drive_theta = avg_drive_theta;
    float current_dist = 0;
    
    float horizontal_error = 0;
    float old_dist = current_dist;
    
    while(fabs(dist-current_dist) > drive_dist_tolerance)
    {
        float drive_error = dist-current_dist;
        left_drive = sign(vIs)*drive_kp*drive_error;
        right_drive = sign(vIs)*drive_kp*drive_error;
        
        left_drive = clamp(left_drive, -fabs(vIs), fabs(vIs));
        right_drive = clamp(right_drive, -fabs(vIs), fabs(vIs));
        
        float heading_error = signedCanonicalizeAngleDeg(target_heading-imu_heading)-horizontal_error;
        float turning_factor = 0.04*heading_error;
        
        if(false)//heading_error > turning_deadband)
        {
            left_drive = -turning_factor;
            right_drive = +turning_factor;
        }
        else
        {
            float drive_factor = (1-clamp(fabs(turning_factor), 0.0, 1.0));
            left_drive *= drive_factor;
            right_drive *= drive_factor;
            left_drive +=  -turning_factor;
            right_drive += +turning_factor;
        }
        
        autonomousUpdate();
        current_dist = sign(vIs)*(avg_drive_theta-start_drive_theta)*sprocket_pitch_radius;
        horizontal_error = (current_dist-old_dist)*sin(heading_error);
        old_dist = current_dist;
    }
    
    right_drive = 0;
    left_drive = 0;        
}

inline void driveOnCourseCm(float dist, float vIs,
                            float target_heading, //Assuming we start facing 180 degrees (intake side)
                            float max_acceleration = default_max_acceleration*cm)
{
    driveOnCourseIn(dist / 2.54, vIs, target_heading, max_acceleration);
}

//TODO: predict when to start decelerating (drive feedforward)
void turnRelDeg(float angle, float vIs)
{
    if(vIs < 0)
    {
        vIs = fabs(vIs);
        angle = -angle;
    }
    
    float target_heading = imu_heading + angle;
    float turning_compensation = 0;
    
    while (fabs(signedCanonicalizeAngleDeg(imu_heading-target_heading)) > acceptableAngleError
           || fabs(imu_heading_omega) > 2)
    {
        float heading_error = signedCanonicalizeAngleDeg(target_heading-imu_heading);
        float turning_factor = turn_kp*heading_error;
        
        if(left_drive_omega < 0.5 && right_drive_omega < 0.5)
        {
            turning_compensation += turn_ki*heading_error*dt;
        }
        else
        {
            turning_compensation = lerp(0.0, turning_compensation, exp(-1.0*dt));
        }
        turning_compensation = clamp(turning_compensation, -2.0, 2.0);
        turning_factor += turning_compensation;
        
        left_drive = -turning_factor;
        right_drive = +turning_factor;
        
        left_drive = clamp(left_drive, -vIs, vIs);
        right_drive = clamp(right_drive, -vIs, vIs);
        autonomousUpdate();
    }
    right_drive = 0;
    left_drive = 0;
}

#endif
