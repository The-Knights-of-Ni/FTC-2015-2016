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

struct trapezoidalMotionProfile
{
    #define max_robot_velocity 18 // in/s
    #define max_robot_acceleration 10 // in/s^2
        //Independent Variables
        float max_velocity;
        float max_acceleration;
        float total_distance;
        float final_velocity;
        //Dependent Variables
        float acceleration_time;
        float distance_while_accelerating;
		float distance_while_deccelerating;
        float distance_while_cruising;
        float target_time;

        trapezoidalMotionProfile(float max_velocity_in, float max_acceleration_in, float total_distance_in, float final_velocity_in)
        {
            //Independent Variables
            max_velocity = min(max_velocity_in, max_robot_velocity);
			max_acceleration = min(max_acceleration_in, max_robot_acceleration);
            total_distance = total_distance_in;
            final_velocity = min(final_velocity_in, max_robot_velocity);
            //Dependent Variables
            acceleration_time = max_velocity/max_acceleration;
            distance_while_accelerating = max_acceleration*sq(acceleration_time)/2.0;//Ignoring v_0 and x_0
            distance_while_deccelerating = max_acceleration*sq(acceleration_time)/2.0;//Ignoring v_0 and x_0, I know decceleration is the wrong term
            distance_while_cruising = total_distance_in - distance_while_deccelerating - distance_while_accelerating;
			//Handles triangle case
            if(distance_while_cruising > 0)
            {
                target_time = 2.0*acceleration_time+distance_while_cruising/max_velocity;
            }
            else
            {
                acceleration_time = sqrt((total_distance/2.0)*(2.0/max_acceleration));
                target_time = 2.0*acceleration_time;
            }

        }
        v3f getData(float drive_time)
        {
            v3f result = (v3f) {0,0,0};
            if(drive_time < acceleration_time) //accelerating
            {
                result[0] = 0.5*max_acceleration*sq(drive_time);
                result[1] = max_velocity*drive_time/acceleration_time;
                result[2] = max_acceleration;
            }
            else if(drive_time > target_time) //stopping
            {
                result[0] = total_distance;
                result[1] = 0;
                result[2] = 0;
            }
            else if(drive_time > target_time-acceleration_time) //deccelerating
            {
                result[0] = total_distance - 0.5*max_acceleration*sq(target_time-drive_time);
                result[1] = max_velocity*(target_time-drive_time)/acceleration_time;
                result[2] = -max_acceleration;
            }
            else //cruising
            {
                result[0] = distance_while_accelerating + max_velocity*(drive_time-acceleration_time);
                result[1] = max_velocity;
                result[2] = 0;
            }
            return result;
        }

};
//TODO: Add filtering, Maximum jerk, and imu stabilization 
void driveDistInFFWD(float dist, float vIs, float max_acceleration = default_max_acceleration)
{
    if (dist < 0.0)
    {
        dist = fabs(dist);
        vIs = -vIs;
    }
    trapezoidalMotionProfile path(vIs*max_robot_velocity, max_robot_velocity, dist, 0);
    float current_dist = 0;
    float error = 0;
    float error_deriv = 0;
    float error_last = 0;
    float drive_time = 0;

    const float Kv = 0.0555;//About 1/18"/s, total guess
    const float Ka = 0.0; //Set to 0 for bashing of Kv
    const float Kp = 0.0; //Error? Nah.
    const float Kd = 0.0;

    //timing or distance, whichever comes later, this is probably better than ignoring time.
    while(drive_time < path.target_time || fabs(dist-current_dist) > drive_dist_tolerance)
    {
        drive_time += dt;
        v3f bot = path.getData(drive_time);
        error = bot.x - current_dist;
        error_deriv = (error - error_last)/dt;
        left_drive = Kv*bot.y + Ka*bot.z + Kp*error + Kd*error_deriv;
        right_drive = Kv*bot.y + Ka*bot.z + Kp*error + Kd*error_deriv;
        error_last = error;
        autonomousUpdate();
    }
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
