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
float imu_heading_omega = 0;
#define imu_tilt (pimu_values->orientation.y)
#define imu_roll (pimu_values->orientation.z)
#define imu_vel (pimu_values->velocity)

void (*customAutonomousUpdate)();

void autonomousUpdate()
{
    dt = time-current_time;
    current_time = time;

    updateDriveSensors();
    updateArmSensors();
    customAutonomousUpdate();
    updateRobot();
    if(imu_heading_omega != imu_heading_omega) imu_heading_omega = 0;
    lowpassFirstDerivativeUpdate(pimu_values->orientation.x/-16.0, &imu_heading, &imu_heading_omega, 10);
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

#define drive_kv 1
#define drive_kp 0.5
#define drive_ki 0.5

#define turn_kp 0.058 //0.1*(slider0/100.0)
#define turn_ki 0.0 //0.05  //0.1*(slider1/100.0)
#define turn_kd 0.0   //0.1*(slider2/100.0)

#define default_max_acceleration 18

#define drive_dist_tolerance 0.1 //1 inch of acceptable error per drive
#define turning_deadband 15 //if there is more than this many degrees of error, the robot will stop driving and just turn

float * pdrive_time = 0;
float * pacceleration_time = 0;

//TODO: go 0-100 for vis instead of 0-1, for clarity //NOTE (Kyler): I like 0-1
void driveDistIn(float dist, float vIs, float max_acceleration = default_max_acceleration)
{
    if (dist < 0)
    {
        dist = fabs(dist);
        vIs *= -1;
    }
    
    #if 1 //feedforward+pid
    
    float max_speed = fabs(vIs)*(neverest_max_omega/drive_gear_ratio)*sprocket_pitch_radius;
    
    float start_drive_theta = avg_drive_theta;
    float current_dist = 0;
    float drive_time = 0;
    
    float acceleration_time = max_speed/max_acceleration;
    float acceleration_dist = (0.5*max_speed*acceleration_time);
    float cruise_dist = dist-2.0*acceleration_dist;
    float target_time = 0;
    
    if(cruise_dist > 0)
    {
        target_time = 2.0*acceleration_time+cruise_dist/max_speed;
    }
    else
    {
        acceleration_time = sqrt((dist/2.0)*(2.0/max_acceleration));
        target_time = 2.0*acceleration_time;
    }
    
    float compensation = 0;
    
    *pdrive_time = target_time;
    *pacceleration_time = acceleration_time;
    
    while(drive_time < target_time ||
          fabs((current_dist - dist) > drive_dist_tolerance)
    {
        drive_time += dt;
        
        //feedforward component
        
        //trapezoidal motion plan
        float desired_acceleration = 0;
        float desired_velocity = 0;
        float desired_displacement = 0;
        if(drive_time < acceleration_time) //accelerating
        {
            desired_displacement = 0.5*max_acceleration*sq(drive_time);
            desired_velocity = max_speed*drive_time/acceleration_time;
            desired_acceleration = +max_acceleration;
        }
        else if(drive_time > target_time) //stopping
        {
            desired_displacement = dist;
            desired_velocity = 0;
            desired_acceleration = 0;
        }
        else if(drive_time > target_time-acceleration_time) //deccelerating
        {
            desired_displacement = acceleration_dist + cruise_dist
                + acceleration_dist -0.5*max_acceleration*sq(target_time-drive_time);
            desired_velocity = max_speed*(target_time-drive_time)/acceleration_time;
            desired_acceleration = -max_acceleration;
        }
        else //cruising
        {
            desired_displacement = acceleration_dist + max_speed*(drive_time-acceleration_time);
            desired_velocity = max_speed;
            desired_acceleration = 0;
        }
        
        float drive_vIs =
            (robot_I*desired_acceleration + (desired_velocity/sprocket_pitch_radius)/(neverest_max_omega/drive_gear_ratio))
            /(neverest_max_torque*4*drive_gear_ratio);
        
        //pid component
        float dist_error = desired_displacement-current_dist;
        drive_vIs += drive_kp*dist_error;
        if(drive_time > target_time)
        {
            compensation += drive_ki*(dist_error)*dt;
            compensation = clamp(compensation, -2*vIs, 2*vIs);
            drive_vIs += compensation;
        }
        else
        {
            compensation = 0;
        }
        
        drive_vIs = sign(vIs)*clamp(drive_vIs, -vIs, vIs); /* this works even when the desired acceleration is
                                                              greater than the max possible motor acceleration
                                                              because the max impulse during the first half
                                                              of the drive will always be the negative max
                                                              impulse during the second half <--this is incorrect,
                                                              it can decellerate faster than it can accelerate */
        left_drive = drive_vIs;
        right_drive = drive_vIs;
        
        current_dist = sign(vIs)*(avg_drive_theta-start_drive_theta)*sprocket_pitch_radius;
        
        autonomousUpdate();
    }
    
    #else
    
    bool doInit = true;
    float start_drive_theta = avg_drive_theta;
    float current_dist = 0;
    
    #if 1 //use right encoder only //TODO: use average?
    while (current_dist < dist)
    {
        right_drive = vIs;
        left_drive = vIs;
        autonomousUpdate();
        current_dist = sign(vIs)*(avg_drive_theta-start_drive_theta)*sprocket_pitch_radius;
    }
    #else //use both encoder
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
        
        if (right_enc_net < dist * encoderticks_per_inch)
        {
            right_drive = vIs;
        }
        if (left_enc_net < dist * encoderticks_per_inch)
        {
            left_drive = vIs;
        }
        
        autonomousUpdate();
    }
    #endif    
    #endif

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
    if (dist < 0)
    {
        dist = fabs(dist);
        vIs = -vIs;
    }

    #if 1 //feedforward+pid for forward, pid for turning
    //TODO: this is copied from the forward only case, compensate for 
    
    float max_speed = vIs*neverest_max_omega/drive_gear_ratio;
    
    float start_drive_theta = avg_drive_theta;
    float current_dist = 0;
    
    float compensation = 0;
    float turning_compensation = 0;
    
    while(fabs((
                   current_dist = sign(vIs)*(avg_drive_theta-start_drive_theta)*sprocket_pitch_radius)
               - dist) > drive_dist_tolerance)
    {
        //feedforward component
        
        //trapezoidal motion plan
        float desired_acceleration = 0;
        float desired_velocity = 0;
        if (current_dist < dist/2)
        {
            float acceleration_dist = 0.5*sq(max_speed)/max_acceleration;
            desired_velocity = current_dist/acceleration_dist*max_speed;
            desired_acceleration = +max_acceleration;
        }
        else
        {
            float acceleration_dist = 0.5*sq(max_speed)/max_acceleration;
            desired_velocity = (dist-current_dist)/acceleration_dist*max_speed;
            desired_acceleration = -max_acceleration;
        }
        if(current_dist*max_acceleration > 0.5*sq(max_speed) || //consv. of mechanical energy
           (dist-current_dist)*max_acceleration > 0.5*sq(max_speed))
        {
            desired_velocity = max_speed;
            desired_acceleration = 0;
        }
        
        //compensate for velocity error
        float velocity_error = desired_velocity-sign(vIs)*avg_drive_omega/sprocket_pitch_radius;
        desired_acceleration += drive_kv*velocity_error; /*TODO; might want to have this completely replace the
                                                           normal acceleration if the velocity error is too great*/
        
        float drive_vIs = (desired_acceleration/sprocket_pitch_radius*robot_I/neverest_max_torque
                           + sign(vIs)*avg_drive_omega/(neverest_max_omega/drive_gear_ratio));
        
        //pid component
        if(fabs(current_dist - dist) < 1.0)
        {
            float dist_error = dist-current_dist;
            compensation += drive_ki*(dist_error)*dt;
            compensation = clamp(compensation, -2*vIs, 2*vIs);
            drive_vIs += compensation + drive_kp*dist_error;
        }
        else
        {
            compensation = 0;
        }
        
        drive_vIs = sign(vIs)*clamp(drive_vIs, -vIs, vIs); /* this works even when the desired acceleration is
                                                              greater than the max possible motor acceleration
                                                              because the max impulse during the first half
                                                              of the drive will always be the negative max
                                                              impulse during the second half <--this is incorrect,
                                                              it can decellerate faster than it can accelerate */
        
        //TODO: account for distance driven to the side        
        float heading_error = signedCanonicalizeAngleDeg(target_heading-imu_heading);
        turning_compensation += turn_ki*heading_error;
        float turning_factor = turn_kp*heading_error + turning_compensation;
        if(heading_error > turning_deadband)
        {
            drive_vIs = 0;
        }
        
        left_drive = drive_vIs - turning_factor;
        right_drive = drive_vIs + turning_factor;
        left_drive = clamp(left_drive, -vIs, vIs);
        right_drive = clamp(right_drive, -vIs, vIs);
        
        autonomousUpdate();
    }
    #else
    #if 1
    float turning_compensation = 0;
    
    float start_drive_theta = avg_drive_theta;
    while((avg_drive_theta-start_drive_theta)*sprocket_pitch_radius < dist)
    {
        float heading_error = target_heading-imu_heading;
        turning_compensation += turn_ki*heading_error;
        float turning_factor = turn_kp*heading_error + turning_compensation;
        
        if(heading_error > turning_deadband)
        {
            left_drive = -turning_factor;
            right_drive = +turning_factor;
        }
        else
        {
            left_drive = vIs - turning_factor;
            right_drive = vIs + turning_factor;
        }
        left_drive = clamp(left_drive, -vIs, vIs);
        right_drive = clamp(left_drive, -vIs, vIs);
        
        autonomousUpdate();
    }
    #else
    
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
            right_prev = right_drive_encoder;
            left_prev = left_drive_encoder;
            doInit = false;
        }
        right_enc_net = right_drive_encoder - right_prev;
        left_enc_net = left_drive_encoder - left_prev;
        if (tolerantEquals(imu_heading, target_heading, acceptableAngleError))
        {
            //TODO: Recovery code that makes sure we don't gain ticks by getting hit
#if 0
            //this might cause us to turn at the very end
            if (right_enc_net < dist * encoderticks_per_cm)
            {
                right_drive = vIs;
            }
            if (left_enc_net < dist * encoderticks_per_cm)
            {
                left_drive = vIs;
            }
#else
            right_drive = vIs;
            left_drive = vIs;
#endif
        }
        else if (isAngleGreaterDeg(imu_heading,
                                   target_heading)) //TODO: make it so it ramps up if we're running at less than 100
        {
            right_drive = vIs;
            left_drive = -vIs;//side_slowing_constant * dt;
        }
        else
        {
            right_drive = -vIs;//side_slowing_constant * dt;
            left_drive = vIs;
        }
        autonomousUpdate();
    }    
    #endif
    #endif
    
    left_drive = 0;
    right_drive = 0;
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
        #if 0 //turn with constant speed
        if (isAngleGreaterDeg(imu_heading, target_heading))
        {
            left_drive = vIs;
            right_drive = -vIs;
        }
        else
        {
            left_drive = -vIs;
            right_drive = vIs;
        }
        #else //turn with proportional control
        float heading_error = signedCanonicalizeAngleDeg(target_heading-imu_heading);
        float turning_factor = turn_kp*heading_error;
        if(turning_factor < 0.1)
        {
            turning_compensation += turn_ki*heading_error*dt;
            turning_factor += turning_compensation;
        }
        else
        {
            turning_compensation = 0;
        }
        
        left_drive = -turning_factor;
        right_drive = +turning_factor;
        
        left_drive = clamp(left_drive, -vIs, vIs);
        right_drive = clamp(right_drive, -vIs, vIs);
        #endif
        autonomousUpdate();
    }
    right_drive = 0;
    left_drive = 0;
}

#endif
