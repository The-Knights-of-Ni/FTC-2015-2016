#ifndef ROBOT_ARM //because it is possible that some compilers will have ARM predefined for the chip archutecture
#define ROBOT_ARM

#include "maths.h"
#include "robotics.h"

int * pelbow_potentiometer;
#define elbow_potentiometer (*pelbow_potentiometer)

int * pshoulder_potentiometer;
#define shoulder_potentiometer (*pshoulder_potentiometer)

float * pshoulder;
#define shoulder (*pshoulder)
float * pwinch;
#define winch (*pwinch)

int * pshoulder_encoder;
#define shoulder_encoder (*pshoulder_encoder)
int * pwinch_encoder;
#define winch_encoder (*pwinch_encoder)

//TODO: tune values
float g = 384; //gravity in " per sec

float winch_pulley_r = 1.0f;
float shoulder_pulley_r = 1.0f;
float elbow_pulley_r = 2.0f;

float winch_gear_ratio = 2.0;
float shoulder_gear_ratio = 6.75;

float forearm_length = 11.5f;
float shoulder_length = 16.5f;

float neverest_max_torque = 4334000; //in g in^2/s^2
float neverest_max_speed = 13.51; //in rad/s

float dc_motor_voltage = 12;

float neverest_k_i = dc_motor_voltage/neverest_max_speed;
float neverest_k_t_over_R = neverest_max_torque/dc_motor_voltage;

//current values
float elbow_potentiometer_angle = 0.0;
float shoulder_potentiometer_angle = 0.0;

float shoulder_compensation = 0; //compensation for constant forces, modified PID I term

float shoulder_theta = 0;
float inside_elbow_theta = 0;
float winch_theta = 0;
bool8 score_mode = true;
float shoulder_omega = 0.0;
float winch_omega = 0.0;
float inside_elbow_omega = 0.0;

bool8 shoulder_active = false;

//target values
float target_arm_theta;
float target_shoulder_theta;
float target_inside_elbow_theta;

void updateArmSensors()
{   
    elbow_potentiometer_angle = (360-((180.0f-potentiometer_range*0.5f+potentiometer_range*(elbow_potentiometer/(1023.0f)))+9.6383f))*pi/180.0f;
    // lerp(
    // (360-((180.0f-potentiometer_range*0.5f+potentiometer_range*(elbow_potentiometer/(1023.0f)))+9.6383f))*pi/180.0f,
    // elbow_potentiometer_angle,
    // exp(-500.0*dt));
    
    shoulder_potentiometer_angle = (-180+((180.0f-potentiometer_range*0.5f+potentiometer_range*(shoulder_potentiometer/(1023.0f)))+125.68f))*pi/180.0f;
    // lerp(
    // (((180.0f-potentiometer_range*0.5f+potentiometer_range*(shoulder_potentiometer/(1023.0f)))+95.047f))*pi/180.0f,
    // shoulder_potentiometer_angle,
    // exp(-500.0*dt));
    
    float new_shoulder_theta = shoulder_potentiometer_angle;//shoulder_encoder/shoulder_gear_ratio/encoderticks_per_radian+pi*150/180.0;
    float new_inside_elbow_theta = elbow_potentiometer_angle;
    float new_winch_theta = winch_encoder/winch_gear_ratio/encoderticks_per_radian;

    if(shoulder_omega != shoulder_omega) shoulder_omega = 0;
    shoulder_omega = lerp((new_shoulder_theta-shoulder_theta)/dt, shoulder_omega, exp(-10*dt));

    if(winch_omega != winch_omega) winch_omega = 0;
    winch_omega = lerp((new_winch_theta-winch_theta)/dt/winch_gear_ratio, winch_omega, exp(-10*dt));
    
    if(inside_elbow_omega != inside_elbow_omega) inside_elbow_omega = 0;
    inside_elbow_omega = lerp((new_inside_elbow_theta-inside_elbow_theta)/dt, inside_elbow_omega, exp(-10*dt));
    
    shoulder_theta = new_shoulder_theta;
    inside_elbow_theta = new_inside_elbow_theta;
    winch_theta = new_winch_theta;
}

float filterArmJoystick(float a)
{
    a = deadzoneAdjust(a);
    if(a == 0) return 0;
    return cubicBezier(a*0.5+0.5, -1.0, 0.0, 0.0, 1.0);
}

void armAtVelocity(v2f target_velocity)
{
    bool8 winch_mode = inside_elbow_theta < (acos((elbow_pulley_r-shoulder_pulley_r)/shoulder_length)+acos(elbow_pulley_r/forearm_length));
    
    ////////////////////////////////////////////////////
    float string_moment_arm;
    
    if(winch_mode)
    {
        float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        
        float string_theta =
            asin(shoulder_length/shoudler_axis_to_end*sin(inside_elbow_theta))
            +asin(shoulder_pulley_r/shoudler_axis_to_end);
        
        string_moment_arm = forearm_length*sin(string_theta);
    }
    else
    {
        string_moment_arm = elbow_pulley_r;
    }
    
    //TODO: fix divides by zero
    float target_winch_omega = 0;
    
    target_velocity.y = filterArmJoystick(target_velocity.y);
    if(target_velocity.y != 0)
    {
        target_winch_omega = target_velocity.y;
        target_inside_elbow_theta = inside_elbow_theta;
    }
    
    float target_inside_elbow_omega = target_winch_omega*winch_pulley_r/string_moment_arm;
    
    float shoulder_axis_to_end_sq = sq(forearm_length)+sq(shoulder_length)
        -2*forearm_length*shoulder_length*cos(inside_elbow_theta);
    float dshoulder_axis_to_end_sq =
        2*forearm_length*shoulder_length*sin(inside_elbow_theta)*target_inside_elbow_omega;
    
    //law of sines: shoulder_theta+arm_theta = asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta))
    float target_shoulder_omega =
        +(  invSqrt(1-sq(forearm_length/sqrt(shoulder_axis_to_end_sq)*sin(inside_elbow_theta)))
            *(  forearm_length/sqrt(shoulder_axis_to_end_sq)*cos(inside_elbow_theta)*target_inside_elbow_omega
                -1.0/2.0*forearm_length*pow(shoulder_axis_to_end_sq, -3.0/2.0)*dshoulder_axis_to_end_sq*sin(inside_elbow_theta)));
    
    target_velocity.x = filterArmJoystick(target_velocity.x);
    shoulder = 0;
    if(target_velocity.x != 0)
    {
        float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
        
        shoulder += target_velocity.x;
        
        shoulder_active = true;
    }
    
    /* if(inside_elbow_theta > pi-0.3) */
    /* { */
    /*     target_winch_omega += 60*(pi-0.3-inside_elbow_theta); */
    /* } */
    /* if(inside_elbow_theta > pi-0.25) */
    /* { */
    /*     //target_winch_omega -= 1000; */
    /*     target_shoulder_omega = target_velocity.x; */
    /*     target_winch_omega = 60*(pi-0.25-inside_elbow_theta)-target_shoulder_omega; */
    /* } */
    
    winch = target_winch_omega+target_shoulder_omega;
    shoulder += shoulder_gear_ratio/winch_gear_ratio*(target_shoulder_omega);
    
    //clamp while keeping ratio constant
    if(winch > 1.0)
    {
        shoulder *= 1.0/winch;
        winch = 1.0;
    }
    if(winch < -1.0)
    {
        shoulder *= -1.0/winch;
        winch = -1.0;
    }
    if(shoulder > 1.0)
    {
        winch *= 1.0/shoulder;
        shoulder = 1.0;
    }
    if(shoulder < -1.0)
    {
        winch *= -1.0/shoulder;
        shoulder = -1.0;
    }
}

void armJointsAtVelocity(v2f target_velocity)
{
    //TODO: this should probably be a different function
    float target_winch_omega = 0;
    
    target_velocity.y = filterArmJoystick(target_velocity.y);
    if(target_velocity.y != 0)
    {
        target_winch_omega = target_velocity.y;
        target_inside_elbow_theta = inside_elbow_theta;
    }
    
    float target_shoulder_omega = 0;
    
    target_velocity.x = filterArmJoystick(target_velocity.x);
    shoulder = 0;
    if(target_velocity.x != 0) //TODO: make the deadzone a constant
    {
        target_shoulder_theta = shoulder_theta;
        
        shoulder = target_velocity.x;
    }
    
    winch = target_winch_omega+target_shoulder_omega;
    shoulder +=
        shoulder_gear_ratio/winch_gear_ratio
        *(target_shoulder_omega);
}

//TEMP
int * pslider0;
#define slider0 (*pslider0)
int * pslider1;
#define slider1 (*pslider1)
int * pslider2;
#define slider2 (*pslider2)
//END TEMP

// the minimum speed the shoulder needs to be rotating for the shoulder to be considered active
static float shoulder_speed_threshold = 0.1;

void armToAngle()
{
    float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
    
    float arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
    
    //TODO: move against the shoulder movement when the shoulder is active for smaller stopping distance
    //TODO: momentum compensation
    //TODO: possible solution to disrupt resonance, detect oscilation and shift reaction time by half a period 
    
    //let the motor stay in neutral until it comes to rest, anti-negative-inertia
    if(shoulder_active)
    {
        shoulder += 0.2*(slider1/100.0)*(neverest_max_speed*shoulder-shoulder_omega); //D
        
        shoulder_active = (fabs(shoulder_omega) > shoulder_speed_threshold);
        target_arm_theta = arm_theta;
        shoulder_compensation = 0;
    }
    else
    {
        float velocity_uncertainty_factor = 0.1;
        
        float shoudler_error = target_arm_theta-arm_theta;
        
        shoulder_compensation *= exp(-velocity_uncertainty_factor*fabs(shoulder_omega)*dt);
        shoulder_compensation += shoudler_error*dt;
        
        shoulder +=
            4*(slider0/100.0)*(shoudler_error) //P
            +0.2*(slider1/100.0)*(neverest_max_speed*shoulder-shoulder_omega) //D
            +10*(slider2/100.0)*shoulder_compensation; //modified I
    }
    winch += 1*(target_inside_elbow_theta-inside_elbow_theta);
}

void armJointsToAngle()
{
    shoulder += 1*(target_shoulder_theta-shoulder_theta);
    winch += 1*(target_inside_elbow_theta-inside_elbow_theta);
}
#endif
