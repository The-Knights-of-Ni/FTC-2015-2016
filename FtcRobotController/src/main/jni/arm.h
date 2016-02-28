#ifndef ROBOT_ARM //because it is possible that some compilers will have ARM predefined for the chip architecture
#define ROBOT_ARM

#include "maths.h"
#include "robotics.h"

float wrist_red_position = 1.0;
float wrist_blue_position = 0.0;
float wrist_level_position = 0.5;

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

//the intake needs to be synchronized with the arm
float * pintake_tilt = 0;
#define intake_tilt (*pintake_tilt)

float * pscore_hook = 0;
#define score_hook (*pscore_hook)

#define tension_switch dim_digital_pin(7)
#define intake_out_switch dim_digital_pin(0)
//#define intake_in_switch dim_digital_pin(1)

//TODO: tune values
float g = 384; //gravity in " per sec

float winch_pulley_r = 1.0f;
float shoulder_pulley_r = 3.0f/25.4f;
float elbow_pulley_r = 2.0f;

float winch_gear_ratio = 2.0;
float shoulder_gear_ratio = 6.75;

float forearm_length = 16.0f;
float shoulder_length = 16.5f;

//current values
float elbow_potentiometer_angle = 0.0;
float shoulder_potentiometer_angle = 0.0;

float shoulder_compensation = 0; //compensation for constant forces, modified PID I term
float winch_compensation = 0; //compensation for constant forces, modified PID I term

float shoulder_theta = 0;
float inside_elbow_theta = 0;
float winch_theta = 0;
bool8 score_mode = true;
float shoulder_omega = 0.0;
float winch_omega = 0.0;
float inside_elbow_omega = 0.0;

bool8 shoulder_active = 0; //0 stabalize mode, 1 stopping, 2 running
bool8 past_shoulder_active = 0;

bool8 forearm_active = 0; //0 stabalize mode, 1 stopping, 2 running
bool8 past_forearm_active = 0;

//target values
float target_arm_theta;
float target_shoulder_theta;
float target_inside_elbow_theta;

#define past_buffers_size 12
float past_shoulder_thetas[past_buffers_size] = {}; //looped buffer
float past_inside_elbow_thetas[past_buffers_size] = {}; //looped buffer
int current_arm_frame = 0;
int n_valid_shoulder_angles = 0;
int n_valid_inside_elbow_angles = 0;

float low_passed_inside_elbow_theta = 0;

void updateArmSensors()
{   
    elbow_potentiometer_angle = (((180.0f-potentiometer_range*0.5f+potentiometer_range*(elbow_potentiometer/(1023.0f)))
                                      -8.050688f))*pi/180.0f;
    // lerp(
    // (360-((180.0f-potentiometer_range*0.5f+potentiometer_range*(elbow_potentiometer/(1023.0f)))+9.6383f))*pi/180.0f,
    // elbow_potentiometer_angle,
    // exp(-500.0*dt));
    
    shoulder_potentiometer_angle = (-90+((180.0f-potentiometer_range*0.5f+potentiometer_range*(shoulder_potentiometer/(1023.0f)))
                                          -2.1190f))*pi/180.0f;
    // lerp(
    // (((180.0f-potentiometer_range*0.5f+potentiometer_range*(shoulder_potentiometer/(1023.0f)))+95.047f))*pi/180.0f,
    // shoulder_potentiometer_angle,
    // exp(-500.0*dt));
    
    float new_shoulder_theta = shoulder_potentiometer_angle;//shoulder_encoder/shoulder_gear_ratio/encoderticks_per_radian+pi*150/180.0;
    float new_inside_elbow_theta = elbow_potentiometer_angle;
    float new_winch_theta = winch_encoder/winch_gear_ratio/encoderticks_per_radian;
    
    if(shoulder_omega != shoulder_omega) shoulder_omega = 0;
    //TODO: switch to use lowpassFirstDerivativeUpdate function
    shoulder_omega = lerp((new_shoulder_theta-shoulder_theta)/dt, shoulder_omega, exp(-10*dt));
    
    if(winch_omega != winch_omega) winch_omega = 0;
    winch_omega = lerp((new_winch_theta-winch_theta)/dt/winch_gear_ratio, winch_omega, exp(-10*dt));
    
    if(inside_elbow_omega != inside_elbow_omega) inside_elbow_omega = 0;
    inside_elbow_omega = lerp((new_inside_elbow_theta-inside_elbow_theta)/dt, inside_elbow_omega, exp(-10*dt));
    
    shoulder_theta = new_shoulder_theta;
    inside_elbow_theta = new_inside_elbow_theta;
    winch_theta = new_winch_theta;
    
    past_shoulder_thetas[current_arm_frame] = shoulder_theta;
    past_inside_elbow_thetas[current_arm_frame] = inside_elbow_theta;
    current_arm_frame = (current_arm_frame+1)%past_buffers_size;
    if(n_valid_shoulder_angles < past_buffers_size) n_valid_shoulder_angles++;
    if(n_valid_inside_elbow_angles < past_buffers_size) n_valid_inside_elbow_angles++;
    
    low_passed_inside_elbow_theta = lerp(inside_elbow_theta, low_passed_inside_elbow_theta, exp(-3*dt));
}

float filterArmJoystick(float a)
{
    a = deadzoneAdjust(a);
    if(a == 0) return 0;
    return cubicBezier(a*0.5+0.5, -1.0, 0.0, 0.0, 1.0);
}

//NOTE: joystick values must be filtered before the arm\(Joints\)?AtVelocity functions are called
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
    float target_shoulder_omega = 0;
    
    if(target_velocity.y != 0)
    {
        target_winch_omega = target_velocity.y;
        target_inside_elbow_theta = inside_elbow_theta;
        low_passed_inside_elbow_theta = inside_elbow_theta;
        
        forearm_active = 2;
        
        //set shoulder to keep aem stable
        float target_inside_elbow_omega = target_winch_omega*winch_pulley_r/string_moment_arm;
        
        float shoulder_axis_to_end_sq = sq(forearm_length)+sq(shoulder_length)
            -2*forearm_length*shoulder_length*cos(inside_elbow_theta);
        float dshoulder_axis_to_end_sq =
            2*forearm_length*shoulder_length*sin(inside_elbow_theta)*target_inside_elbow_omega;
        
        //law of sines: shoulder_theta-arm_theta = asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta))
        target_shoulder_omega +=
            +(  invSqrt(1-sq(forearm_length/sqrt(shoulder_axis_to_end_sq)*sin(inside_elbow_theta)))
                *(  forearm_length*invSqrt(shoulder_axis_to_end_sq)*cos(inside_elbow_theta)*target_inside_elbow_omega
                    -1.0/2.0*forearm_length*pow(shoulder_axis_to_end_sq, -3.0/2.0)*dshoulder_axis_to_end_sq*sin(inside_elbow_theta)));
        
        if(target_shoulder_omega < 0.2) target_shoulder_omega = 0;
    }
    
    shoulder = 0;
    if(target_velocity.x != 0)
    {
        float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
        
        shoulder += target_velocity.x;
        
        shoulder_active = 2;
    }
    
    shoulder += shoulder_gear_ratio/winch_gear_ratio*(target_shoulder_omega);
    winch = target_winch_omega/* +shoulder*(winch_gear_ratio*shoulder_gear_ratio) */;
    
    //clamp while keeping ratio constant
    /* if(winch > 1.0) */
    /* { */
    /*     shoulder *= 1.0/winch; */
    /*     winch = 1.0; */
    /* } */
    /* if(winch < -1.0) */
    /* { */
    /*     shoulder *= -1.0/winch; */
    /*     winch = -1.0; */
    /* } */
    /* if(shoulder > 1.0) */
    /* { */
    /*     winch *= 1.0/shoulder; */
    /*     shoulder = 1.0; */
    /* } */
    /* if(shoulder < -1.0) */
    /* { */
    /*     winch *= -1.0/shoulder; */
    /*     shoulder = -1.0; */
    /* } */
}

void armJointsAtVelocity(v2f target_velocity)
{
    float target_winch_omega = 0;
    
    if(target_velocity.y != 0)
    {
        target_winch_omega = target_velocity.y;
        target_inside_elbow_theta = inside_elbow_theta;
        
        low_passed_inside_elbow_theta = inside_elbow_theta;
        
        forearm_active = 2;
    }
    
    shoulder = 0;
    if(target_velocity.x != 0)
    {
        target_shoulder_theta = shoulder_theta;
        
        shoulder += target_velocity.x;
        
        shoulder_active = 2;
    }
    
    winch = target_winch_omega/* +winch_gear_ratio/shoulder_gear_ratio*shoulder */;
}

//TEMP
int * pslider0;
#define slider0 (*pslider0)
int * pslider1;
#define slider1 (*pslider1)
int * pslider2;
#define slider2 (*pslider2)
int * pslider3;
#define slider3 (*pslider3)
//END TEMP

//TODO: might need to retune for new arm?
#define shoulder_kp    0.84  //(4*(slider0/100.0))
#define shoulder_kd    0.036 //(0.2*(slider1/100.0))
#define shoulder_ki    0.9   //(10*(slider2/100.0))
#define shoulder_kslow 0.15  //(1.0*(slider3/100.0))
// the minimum speed the shoulder needs to be rotating for the shoulder to be considered active
static const float shoulder_speed_threshold = 0.1;

//TODO: actually tune these? work fairly well, but most of these are the values from the old arm
#define winch_kp    1.5  //(4*(slider0/100.0))
#define winch_kd    0.036 //(0.2*(slider1/100.0))
#define winch_ki    1.7 //0.9   //(10*(slider2/100.0))
#define winch_kslow 0.15  //(1.0*(slider3/100.0))
static const float inside_elbow_speed_threshold = 0.1;

#define momentum_compensation_speed_threshold 7
#define momentum_compensation_angle_threshold pi/2

void armJointStabalizationFunction(float * motor,
                                   float joint_theta, float omega_error,
                                   bool8 * joint_active, float * joint_compensation,
                                   float * past_thetas, int * n_valid_angles, float speed_threshold,
                                   float kp, float kd, float ki, float kslow,
                                   float * target_theta,
                                   bool8 stabalize_while_running)
{
    if(*joint_active) //try to bring the arm to a stop, or to the target velocity if the joystick is pressed
    {
        float min_theta = past_thetas[0];
        float max_theta = past_thetas[0];
        for(int i = 1; i < past_buffers_size; i++)
        {
            if(past_thetas[i] < min_theta) min_theta = past_thetas[i];
            if(past_thetas[i] > max_theta) max_theta = past_thetas[i];
        }
        
        if(*joint_active == 2)
        {
            *n_valid_angles = 0;
            *joint_compensation = 0;//clamp(*motor, -0.0, 0.1);
        }
        
        if(*joint_active != 2 || stabalize_while_running)
        {
            if(*n_valid_angles >= past_buffers_size
               && fabs(max_theta-min_theta) < speed_threshold)
            { //if joint is still then transition to stabalization mode
                *joint_active = 0;
            }
            else
            { //otherwise try to stop the joint
                *joint_compensation += (-omega_error)*kslow*dt;
                
                *motor +=
                    kd*(omega_error)
                    +(*joint_compensation);
                
                *target_theta = joint_theta;
            }
        }
        if((*joint_active) == 2)
        {
            *joint_active = 1;
        }
    }
    
    if(*joint_active == 0)
    {
        float theta_error = (*target_theta)-joint_theta;
        
        *joint_compensation += ki*theta_error*dt;
        
        *motor += 1*theta_error;
        *motor +=
            kp*(theta_error) //P
            +kd*(omega_error) //D
            +(*joint_compensation); //modified I
    }
}

inline void armToAngle(float * target_theta, float current_theta)
{
    float shoulder_omega_error = (neverest_max_omega*shoulder-shoulder_omega);
    
    armJointStabalizationFunction(&shoulder,
                                  current_theta, shoulder_omega_error,
                                  &shoulder_active, &shoulder_compensation,
                                  past_shoulder_thetas, &n_valid_shoulder_angles, shoulder_speed_threshold,
                                  shoulder_kp, shoulder_kd, shoulder_ki, shoulder_kslow,
                                  target_theta,
                                  true);
    
    /* armJointStabalizationFunction(&winch, */
    /*                               inside_elbow_theta, -inside_elbow_omega, */
    /*                               &forearm_active, &winch_compensation, */
    /*                               past_inside_elbow_thetas, &n_valid_inside_elbow_angles, inside_elbow_speed_threshold, */
    /*                               winch_kp, winch_kd, winch_ki, winch_kslow, */
    /*                               &target_inside_elbow_theta, */
    /*                               false); */
}

void armToPolarTarget()
{
    float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
    
    float arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
    
    armToAngle(&target_arm_theta, arm_theta);
}

void armToJointTarget()
{
    armToAngle(&target_shoulder_theta, shoulder_theta);
}

enum arm_stage_enum
{
    arm_idle,

    arm_pre_preparing,
    arm_preparing,
    arm_lowering,
    arm_extending,
    
    arm_retracting,
    arm_raising,
    arm_going_to_score_position
};
int arm_stage = 0;

bool8 arm_running_to_pos = 0;
bool8 old_shoulder_side = 0;
bool8 passed_shoulder_target = 0;
bool8 old_inside_elbow_side = 0;
bool8 passed_inside_elbow_target = 0;

bool8 armIsAtTarget(float shoulder_tolerance, float inside_elbow_tolerance)
{
    #if 0
    bool8 at_target = false;
    
    if(!arm_running_to_pos)
    {
        arm_running_to_pos = true;
        passed_shoulder_target = false;
        passed_inside_elbow_target = false;
    }
    else
    {
        if(!passed_shoulder_target)
        {
            passed_shoulder_target = (old_shoulder_side) != (shoulder_theta > target_shoulder_theta);
        }
                
        if(!passed_inside_elbow_target)
        {
            passed_inside_elbow_target = (old_inside_elbow_side) != (inside_elbow_theta > target_inside_elbow_theta);
        }
        
        at_target = passed_shoulder_target && passed_inside_elbow_target;
    }
    
    old_inside_elbow_side = (inside_elbow_theta > target_inside_elbow_theta);
    old_shoulder_side = (shoulder_theta > target_shoulder_theta);

    if(at_target) arm_running_to_pos = false;
    return at_target;

    #else
    
    //const float angle_tolerance = 0.1;
    const float speed_tolerance = 0.25;
    
    return (fabs(inside_elbow_theta-target_inside_elbow_theta) < shoulder_tolerance &&
            /* fabs(inside_elbow_omega) < speed_tolerance && */
            fabs(shoulder_theta-target_shoulder_theta)         < inside_elbow_tolerance /* && */
            /* fabs(shoulder_omega)     < speed_tolerance */);
    #endif
}

void armToPreset(float shoulder_speed, float winch_speed, float tolerance = 0.5)
{
    float shoulder_error = target_shoulder_theta-shoulder_theta;
    
    if     (shoulder_theta > target_shoulder_theta+tolerance){ shoulder = -shoulder_speed; shoulder_active = 2; shoulder_compensation = 0;}
    else if(shoulder_theta < target_shoulder_theta-tolerance){ shoulder = +shoulder_speed; shoulder_active = 2; shoulder_compensation = 0;}
    else
    {
        shoulder_compensation += shoulder_ki*shoulder_error*dt;
        shoulder = 3*shoulder_error+shoulder_compensation;
    }
    
    float inside_elbow_error = target_inside_elbow_theta-inside_elbow_theta;
    
    if     (inside_elbow_theta > target_inside_elbow_theta+tolerance){ winch = -winch_speed; forearm_active = 2; winch_compensation = 0;}
    else if(inside_elbow_theta < target_inside_elbow_theta-tolerance){ winch = +winch_speed; forearm_active = 2; winch_compensation = 0;}
    else
    {
        winch_compensation += winch_ki*inside_elbow_error*dt;
        winch = winch_kp*(inside_elbow_error)+winch_compensation;
    }
}

#define intake_swing_time 1.3

bool8 intake_out = 0;
float intake_time = intake_swing_time;
void doIntake()
{
    intake_time += dt;
    if(intake_time < intake_swing_time)
    {
        if(intake_out) intake_tilt = 1.0;
        else           intake_tilt = 0.0;
    }
    else intake_tilt = continuous_servo_stop;
}

void setIntakeIn()
{
    if(intake_out)
    {
        intake_out = false;
        intake_time = 0;
    }
}

void setIntakeOut()
{
    if(!intake_out)
    {
        intake_out = true;
        intake_time = 0;
    }
}


#define hand_open_time 0.5

bool8 hand_open = 0;
float hand_time = 0;
void doHand()
{
    if(hand_open) hand = 1.0;
    else          hand = 0.0;
    hand_time += dt;
    /* if(hand_time < hand_open_time) */
    /* { */
    /*     if(hand_open) hand = 1.0; */
    /*     else hand = 0.0; */
    /* } */
    /* else hand = continuous_servo_stop; */
}

void setHandShut()
{
    if(hand_open)
    {
        hand_open = false;
        hand_time = 0;
    }
}

void setHandOpen()
{
    if(!hand_open)
    {
        hand_open = true;
        hand_time = 0;
    }
}

int double_close_case = 0;
bool8 double_close_hand_completed = 0;
void doubleCloseHand()
{
    //TODO: handle being interupted
    if(double_close_case == 0)
    {
        hand_open = false;
        hand_time = 0;
        double_close_case++;
    }
    switch(double_close_case)
    {
        case 1:
        {
            if(hand_time > hand_open_time)
            {
                setHandOpen();
                double_close_case++;
            }
        } break;
        case 2:
        {
            if(hand_time > hand_open_time)
            {
                setHandShut();
                double_close_case++;
            }
        } break;
        case 3:
        {
            if(hand_time > hand_open_time)
            {
                double_close_case = 0;
                double_close_hand_completed = true;
            }
        } break;
    }
}

#define shoulder_mode_threshold 1.9

bool8 armOnIntakeSide()
{
    return shoulder_theta > shoulder_mode_threshold && inside_elbow_theta > pi;
}

int arm_mode = 0;
bool8 arm_switching = 0;
float arm_time = 0;
void armSwitchModes()
{
    shoulder = 0;
    winch = 0;
    
    #define arm_case(stage) case arm_##stage: arm_##stage##_case
    #define goto_arm_case(stage) {arm_stage = arm_##stage; goto arm_##stage##_case;}
    
    //TODO(eventually): just setup map of waypoints and connections and use A* to find the path to a desired target position
    
    //TODO: tune waypoints
    
    arm_time += dt;
    
    switch(arm_stage)
    {
        //ENTERING INTAKE
        arm_case(pre_preparing):
        {
            if(!arm_switching)
            {
                arm_switching = true;
                
                arm_time = 0;
                if(arm_mode)
                {
                    arm_mode = 0;
                    intake_out = true;
                    intake_time = 0;
                }
                else
                {
                    //skip pre-preparing if we're already in intake mode
                    goto_arm_case(preparing);
                }
            }
            
            //setIntakeOut();
            setHandShut();
            
            arm_mode = 0;
            target_shoulder_theta = 2.0;
            target_inside_elbow_theta = 4.0;
            armToPreset(1.0, 1.0);
            
            if(armIsAtTarget(0.20, 0.20) && arm_time > 0.7 /* && intake_time > intake_swing_time */) goto_arm_case(preparing);
        } break;
        
        arm_case(preparing):
        {            
            setIntakeOut();
            setHandShut();
            
            target_shoulder_theta = 2.2;//1.86;
            target_inside_elbow_theta = 4.3;
            
            armToPreset(0.8, 0.5, 0.25);
            
            if(armIsAtTarget(0.1, 0.1) && hand_time > hand_open_time) goto_arm_case(lowering);
        } break;
        
        arm_case(lowering):
        {
            target_shoulder_theta = 2.35;
            target_inside_elbow_theta = 4.4;
            
            armToPreset(0.1, 0.5);
            
            setIntakeOut();
            setHandShut();
            if(armIsAtTarget(0.1, 0.1)) goto_arm_case(extending);
        } break;
        
        arm_case(extending):
        {
            target_shoulder_theta = 2.46;
            target_inside_elbow_theta = 4.43;
            
            armToPreset(0.1, 0.5);
            
            setIntakeOut();
            setHandShut();
            if(armIsAtTarget(0.1, 0.1))
            {
                setHandOpen();
                arm_switching = false;
                arm_stage = arm_idle;
            }
        } break;
        
        //EXITING INTAKE
        arm_case(retracting):
        {
            if(!arm_mode)
            {
                arm_mode = 1;
                double_close_hand_completed = false;
            }
            
            if(!armOnIntakeSide())
            {
                setIntakeIn();
                goto_arm_case(going_to_score_position);
            }
            
            //setIntakeOut();
            setHandShut();
            if(hand_time > hand_open_time)
            {
                target_shoulder_theta = 1.8;
                target_inside_elbow_theta = 4.38;
                
                armToPreset(1.0, 1.0);
                
                //doubleCloseHand();
                
                if(armIsAtTarget(0.25, 0.25) /* && double_close_hand_completed */) goto_arm_case(raising);
            }
            else
            {
                armToPreset(1.0, 1.0);
            }
        } break;
        
        arm_case(raising):
        {
            target_shoulder_theta = 2.0;
            target_inside_elbow_theta = pi*2/3;
            
            armToPreset(1.0, 1.0);
            
            if(armIsAtTarget(0.25, 0.25)) {arm_stage = arm_idle; setIntakeIn();}
        } break;
        
        arm_case(going_to_score_position):
        {
            target_shoulder_theta = 1.0;
            target_inside_elbow_theta = pi*7/8;
            
            armToPreset(1.0, 1.0, 1.0);
            
            float shoudler_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                              -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
            target_arm_theta = shoulder_theta-asin(forearm_length/shoudler_axis_to_end*sin(inside_elbow_theta));
            
            if(armIsAtTarget(0.15, 0.25)){arm_stage = arm_idle; setIntakeIn();}
        } break;
    }
    
    #undef arm_case
    #undef goto_arm_case
}

#endif
