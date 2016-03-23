#ifndef ROBOT_ARM //because it is possible that some compilers will have ARM predefined for the chip architecture
#define ROBOT_ARM

#include "maths.h"
#include "robotics.h"

bool8 suppress_arm = 0; //used in auto only

float wrist_red_position = 1.0;
float wrist_blue_position = 0.0;
float wrist_level_position = 0.5;

bool8 wrist_tilt = 0;
float wrist_manual_control = 0;

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

int * pintake_potentiometer;
#define intake_potentiometer (*pintake_potentiometer)

int * pwrist_potentiometer;
#define wrist_potentiometer (*pwrist_potentiometer)

float * pscore_hook = 0;
#define score_hook (*pscore_hook)

#define tension_switch dim_digital_pin(7)

float winch_pulley_r = 1.0f;
float shoulder_pulley_r = 3.0f/25.4f;
float elbow_pulley_r = 2.0f;

float winch_gear_ratio = 2.0;
float shoulder_gear_ratio = 6.75;

float forearm_length = 16.0f; //TODO: this might have changed
float shoulder_length = 16.5f;

float intake_out_theta = -0.28;
float intake_in_theta = 0.9;//1.89;

//sliders
int * pslider0;
#define slider0 (*pslider0)
int * pslider1;
#define slider1 (*pslider1)
int * pslider2;
#define slider2 (*pslider2)
int * pslider3;
#define slider3 (*pslider3)

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

bool8 forearm_active = 0; //0 stabalize mode, 1 stopping, 2 running

float intake_potentiometer_angle;

float intake_theta = 0.0;
float intake_omega = 0.0;

float wrist_potentiometer_angle;

float wrist_theta = 0.0;
float wrist_omega = 0.0;

int arm_line = 0;
float arm_time = 0;

//target values
float target_wrist_theta = 0;

float target_intake_theta = 0;

const float vertical_arm_theta = pi/2;

float target_arm_theta;
float target_arm_y;
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
    
    log("I");
    shoulder_potentiometer_angle = (-90+((180.0f-potentiometer_range*0.5f+potentiometer_range*(shoulder_potentiometer/(1023.0f)))
                                         -2.1190f))*pi/180.0f;
    
    log("I");
    if(shoulder_omega != shoulder_omega) shoulder_omega = 0;
    lowpassFirstDerivativeUpdate(shoulder_potentiometer_angle, &shoulder_theta, &shoulder_omega, 138);
    
    log("I");
    if(winch_omega != winch_omega) winch_omega = 0;
    lowpassFirstDerivativeUpdate(winch_encoder/winch_gear_ratio/encoderticks_per_radian, &winch_theta, &winch_omega, 138);
    
    log("I");
    if(inside_elbow_omega != inside_elbow_omega) inside_elbow_omega = 0;
    lowpassFirstDerivativeUpdate(elbow_potentiometer_angle, &inside_elbow_theta, &inside_elbow_omega, 138);
    
    log("I");
    past_shoulder_thetas[current_arm_frame] = shoulder_theta;
    past_inside_elbow_thetas[current_arm_frame] = inside_elbow_theta;
    current_arm_frame = (current_arm_frame+1)%past_buffers_size;
    if(n_valid_shoulder_angles < past_buffers_size) n_valid_shoulder_angles++;
    if(n_valid_inside_elbow_angles < past_buffers_size) n_valid_inside_elbow_angles++;
    
    log("I");
    low_passed_inside_elbow_theta = lerp(inside_elbow_theta, low_passed_inside_elbow_theta, exp(-17*dt));
    
    log("I");
    //intake
    intake_potentiometer_angle = -(potentiometer_range*(intake_potentiometer-646.0)/(1023.0f))*pi/180.0f;     
    if(intake_omega != intake_omega) intake_omega = 0;
    lowpassFirstDerivativeUpdate(intake_potentiometer_angle, &intake_theta, &intake_omega, 138);
    
    log("I");
    //wrist
    wrist_potentiometer_angle = -(potentiometer_range*(wrist_potentiometer-505.0)/(1023.0f))*pi/180.0f;
    
    log("I");
    if(wrist_omega != wrist_omega) wrist_omega = 0;
    lowpassFirstDerivativeUpdate(wrist_potentiometer_angle, &wrist_theta, &wrist_omega, 138);
    log("I\n");
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
        float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        
        float string_theta =
            asin(shoulder_length/shoulder_axis_to_end*sin(inside_elbow_theta))
            +asin(shoulder_pulley_r/shoulder_axis_to_end);
        
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
            //due to radius change
            #if 0
            (  sign(shoulder_theta-vertical_arm_theta)*sin(target_arm_y*invSqrt(shoulder_axis_to_end_sq))
               *(  -1.0/2.0*target_arm_y*pow(shoulder_axis_to_end_sq, -3.0/2.0)*dshoulder_axis_to_end_sq))
            #endif
            //due to elbow bending
            +(  invSqrt(1-sq(forearm_length/sqrt(shoulder_axis_to_end_sq)*sin(inside_elbow_theta)))
                *(  forearm_length*invSqrt(shoulder_axis_to_end_sq)*cos(inside_elbow_theta)*target_inside_elbow_omega
                    -1.0/2.0*forearm_length*pow(shoulder_axis_to_end_sq, -3.0/2.0)*dshoulder_axis_to_end_sq*sin(inside_elbow_theta)));
        
        //if(target_shoulder_omega < 0.2) target_shoulder_omega = 0;
    }
    
    shoulder = 0;
    if(target_velocity.x != 0)
    {
        float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
        target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
        
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

#define shoulder_kp 0.650000 //(1*(slider0/100.0))
#define shoulder_kd 0.110000 //(0.2*(slider1/100.0))
#define shoulder_ki 0.790000 //(1*(slider2/100.0))
/* #define shoulder_kp    0.84  //(4*(slider0/100.0)) */
/* #define shoulder_kd    0.1  //0.036 //(0.2*(slider1/100.0)) */
/* #define shoulder_ki    0.9   //(10*(slider2/100.0)) */
#define shoulder_kslow 0.15  //(1.0*(slider3/100.0))
// the minimum speed the shoulder needs to be rotating at for the shoulder to be considered active
static const float shoulder_speed_threshold = 0.1;

//TODO: actually tune these? works fairly well, but most of these are the values from the old arm
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
            
            #if 0
            #else //this was what was running at the tournament
            *joint_compensation = 0;
            #endif
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
            +(*joint_compensation); //I
    }
}

inline void armToAngle(float * target_theta, float current_theta)
{
    float shoulder_omega_error = (neverest_max_omega/shoulder_gear_ratio*shoulder-shoulder_omega);
    
    //TODO: merge all the parameters only used here into a struct
    armJointStabalizationFunction(&shoulder,
                                  current_theta, shoulder_omega_error,
                                  &shoulder_active, &shoulder_compensation,
                                  past_shoulder_thetas, &n_valid_shoulder_angles, shoulder_speed_threshold,
                                  shoulder_kp, shoulder_kd, shoulder_ki, shoulder_kslow,
                                  target_theta,
                                  true);

    if(forearm_active == 2)
    {
        winch_compensation = 0;
        forearm_active = 0;
    }
    else
    {
        float inside_elbow_error = target_inside_elbow_theta-low_passed_inside_elbow_theta;
        
        //winch_compensation += 2.28*inside_elbow_error*dt;
        winch = 0.99*(inside_elbow_error)-0.2*inside_elbow_omega;
    }
    /* armJointStabalizationFunction(&winch, */
    /*                               inside_elbow_theta, -inside_elbow_omega, */
    /*                               &forearm_active, &winch_compensation, */
    /*                               past_inside_elbow_thetas, &n_valid_inside_elbow_angles, inside_elbow_speed_threshold, */
    /*                               winch_kp, winch_kd, winch_ki, winch_kslow, */
    /*                               &target_inside_elbow_theta, */
    /*                               false); */
}

void armToGoalOrientedTarget()
{
    float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
    
    float arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));

    target_arm_theta = acos(target_arm_y/shoulder_axis_to_end)+vertical_arm_theta;
    
    armToAngle(&target_arm_theta, arm_theta);
}

float arm_tilt_adjustment = 0;
float arm_tilt_omega_adjustment = 0;
#define min_arm_imu_tilt 25.0

void armIMUOmegaStabilize()
{
    /* float instantaneous_arm_tilt_omega_adjustment = (imu_tilt_omega*pi/180.0)*shoulder_gear_ratio/neverest_max_omega; */
    /* float low_passed_arm_tilt_omega_adjustment = lerp(instantaneous_arm_tilt_omega_adjustment, */
    /*                                                   arm_tilt_omega_adjustment, */
    /*                                                   exp(-15.5*dt)); */
    /* if(fabs(instantaneous_arm_tilt_omega_adjustment) < fabs(low_passed_arm_tilt_omega_adjustment)) */
    /* { */
    /*     arm_tilt_omega_adjustment = instantaneous_arm_tilt_omega_adjustment; */
    /* } */
    /* else */
    /* { */
    /*     arm_tilt_omega_adjustment = low_passed_arm_tilt_omega_adjustment; */
    /* } */

    
    /* if(fabs(imu_tilt) > min_arm_imu_tilt) */
    /* {         */
    /*     shoulder += arm_tilt_omega_adjustment; */
    /* } */
}

float armIMUThetaAdjustment()
{
    return 0;//lerp(-imu_tilt*pi/180.0, arm_tilt_adjustment, exp(-(slider0)*dt));
}

void armToPolarTarget(float shoulder_power = 1.0, float winch_power = 1.0)
{
    float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                      -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
    
    float arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
    
    arm_tilt_adjustment = armIMUThetaAdjustment();
    
    armToAngle(&target_arm_theta, arm_theta+arm_tilt_adjustment);
    
    shoulder = clamp(shoulder, -1.0, 1.0);
    winch = clamp(winch, -1.0, 1.0);
    shoulder *= shoulder_power;
    winch *= winch_power;
    shoulder = clamp(shoulder, -1.0, 1.0);
    winch = clamp(winch, -1.0, 1.0);
    
    armIMUOmegaStabilize();
}

void armToJointTarget()
{
    armToAngle(&target_shoulder_theta, shoulder_theta);
}

bool8 armIsAtTarget(float shoulder_tolerance, float inside_elbow_tolerance)
{
    return (fabs(inside_elbow_theta-target_inside_elbow_theta) < shoulder_tolerance &&
            fabs(shoulder_theta-target_shoulder_theta)         < inside_elbow_tolerance);
}

void armToPreset(float shoulder_speed, float winch_speed, float tolerance = 0.5, float winch_preset_kp = 0.99, float winch_preset_ki = 0.9, float shoulder_preset_kp = 1.98, float shoulder_preset_ki = 0.53)
{
    float shoulder_error = target_shoulder_theta-shoulder_theta;
    
    if     (shoulder_theta > target_shoulder_theta+tolerance){ shoulder = -shoulder_speed; shoulder_active = 2; shoulder_compensation = 0;}
    else if(shoulder_theta < target_shoulder_theta-tolerance){ shoulder = +shoulder_speed; shoulder_active = 2; shoulder_compensation = 0;}
    else
    {
        //2.4, .69, .25
        //1.98, .18, .20
        shoulder_compensation += shoulder_preset_ki*shoulder_error*dt;
        shoulder = shoulder_preset_kp*shoulder_error+shoulder_compensation-0.2*shoulder_omega;
    }
    
    float inside_elbow_error = target_inside_elbow_theta-low_passed_inside_elbow_theta;
    
    if     (inside_elbow_theta > target_inside_elbow_theta+tolerance){ winch = -winch_speed; forearm_active = 2; winch_compensation = 0;}
    else if(inside_elbow_theta < target_inside_elbow_theta-tolerance){ winch = +winch_speed; forearm_active = 2; winch_compensation = 0;}
    else
    {
        //.67, .57, .10
        if(tension_switch)
        {
            /* winch_compensation += 2.28*inside_elbow_error*dt; */
            /* winch = 2.68*(inside_elbow_error)+winch_compensation-0.2*inside_elbow_omega; */
            winch_compensation += winch_preset_ki*inside_elbow_error*dt;
            winch = winch_preset_kp*(inside_elbow_error)+winch_compensation-0.2*inside_elbow_omega;
        }
        else
        {
            winch = -0.1;
        }
    }
}

void armToPresetWithIMU(float shoulder_speed, float winch_speed, float tolerance = 0.5)
{
    float temp_target_shoulder_theta = target_shoulder_theta;
    target_shoulder_theta -= armIMUThetaAdjustment();
    
    armToPreset(shoulder_speed, winch_speed, tolerance);
    
    target_shoulder_theta = temp_target_shoulder_theta;
    armIMUOmegaStabilize();
}

#define intake_swing_time 1.3

bool intake_out = 0;
float old_target_intake_theta = 0;
bool intake_running = 0;
float intake_stopped_timer = 0;

void doIntake()
{
    if(fabs(old_target_intake_theta - target_intake_theta))
    {
        intake_running = true;
        old_target_intake_theta = target_intake_theta;
        intake_stopped_timer = 0;
    }
    
    intake_tilt = continuous_servo_stop;

    if(intake_running)
    {
        if(fabs(intake_omega) < 0.1)
        {
            intake_stopped_timer += dt;
        }
        else
        {
            intake_stopped_timer = 0.0;
        }
        
        if(fabs(target_intake_theta - intake_theta) < 0.1 || intake_stopped_timer > 0.25)
        {
            intake_running = false;
            target_intake_theta = intake_theta;
            return;
        }
        
        intake_tilt += 0.8*(target_intake_theta-intake_theta);
    }
}

void setIntakeIn()
{
    intake_out = 0;
    target_intake_theta = intake_in_theta;
}

void setIntakeOut()
{
    intake_out = 1;
    target_intake_theta = intake_out_theta;
}

#define wrist_swing_time 0.5
float wrist_time = 0;

void doWrist()
{
    wrist_time += dt;
    
    if(wrist_tilt)
    {
        if(current_color) target_wrist_theta = 1.396; //wrist_red_position;
        else              target_wrist_theta = -1.396; //wrist_blue_position;
        target_wrist_theta += wrist_manual_control;
    }
    else
    {
        target_wrist_theta = 0.0;
        wrist_manual_control = 0;
    }
    
    wrist = continuous_servo_stop;
    wrist += 1.5*(target_wrist_theta-wrist_theta)-0.2*wrist_omega;
    if(wrist < 0.1) wrist = 0;
    wrist += wrist_manual_control;
}

void setWristIn()
{
    if(wrist_tilt)
    {
        wrist_tilt = false;
        wrist_time = 0;
    }
}

void setWristOut()
{
    if(!wrist_tilt)
    {
        wrist_tilt = true;
        wrist_time = 0;
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

#define shoulder_mode_threshold 1.9

bool8 armOnIntakeSide()
{
    return shoulder_theta > shoulder_mode_threshold && inside_elbow_theta > pi;
}

void (*armFunction)();

v2f target_arm_velocity;

void armAutonomousControl()
{
    armToPreset(1.0, 1.00);
}

void armUserControl()
{
    if(score_mode)
    {
        armAtVelocity(target_arm_velocity);
            
        //set the unused target angle so it won't move when the mode is switched
        target_shoulder_theta = shoulder_theta;
        
        //armToGoalOrientedTarget();
        armToPolarTarget();
    }
    else
    {
        armJointsAtVelocity(target_arm_velocity);
        
        //set the unused target angle so it won't move when the mode is switched
        float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
        target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
        
        armToJointTarget();
    }
}

//NOTE: this might make it more difficult for the compiler to optimize the switch, since the cases will not be sequential
//NOTE: this can only be used in the functions with an arm_line switch

#define switchTasks() arm_line = __LINE__; return; case __LINE__:

void armToIntakeMode()
{
    shoulder = 0;
    winch = 0;
    
    switch(arm_line)
    {
        case 0:default:
        {}
        
        setIntakeOut();
        setHandShut();
        setWristIn();
        
        while(fabs(target_wrist_theta-wrist_theta) > 0.1 || intake_running)
        {
            armToPreset(1.0, 1.0);
            switchTasks();
        }
        
        /* arm_time = 0; */
        /* while(arm_time < 0.5) */
        /* { */
        /*     arm_time += dt; */
        /*     armToPreset(1.0, 1.0); */
        /*     switchTasks(); */
        /* } */
        
        if(score_mode) //is in score mode
        {
            score_mode = 0;
            
            target_shoulder_theta = 2.0;
            target_inside_elbow_theta = pi*2.0/3.0;
            while(!armIsAtTarget(0.10, 0.10))
            {
                armToPreset(1.0, 1.0, 1.0);
                
                switchTasks();
            }
            
            arm_time = 0;
            
            while(arm_time < 0.1)
            {
                arm_time += dt;
                armToPreset(1.0, 1.0);
                switchTasks();
            }
        }
        
        target_shoulder_theta = 2.2;
        target_inside_elbow_theta = 4.3;
        while(!armIsAtTarget(0.1, 0.1) || hand_time < hand_open_time)
        {
            armToPreset(0.8, 0.5, 0.5, 2.68, 2.28, 2.61, 0.8);
            
            switchTasks();
        }
        
        target_shoulder_theta = 2.35;
        target_inside_elbow_theta = 4.4;
        while(!armIsAtTarget(0.1, 0.1))
        {
            armToPreset(1.0, 0.5, 0.5, 2.68, 2.28, 2.61, 0.8);
            
            switchTasks();
        }
        
        target_shoulder_theta = 2.46;
        target_inside_elbow_theta = 4.43;
        while(!armIsAtTarget(0.1, 0.1))
        {
            armToPreset(1.0, 0.5, 0.5, 2.68, 2.28, 2.61, 0.8);
            
            switchTasks();
        }
        
        arm_time = 0;
            
        while(!armIsAtTarget(0.1, 0.1) || arm_time < 0.2)
        {
            arm_time += dt;
            
            armToPreset(1.0, 0.5, 0.5, 2.68, 2.28, 2.61, 0.8);
            
            switchTasks();
        }
        
        setHandOpen();
        
        armFunction = armUserControl;
    }
}


void armToScoreMode()
{
    shoulder = 0;
    winch = 0;
    
    switch(arm_line)
    {
        case 0:default:
        {}
        
        score_mode = 1;
        
        if(armOnIntakeSide())
        {
            setHandShut();
            
            while(hand_time < hand_open_time)
            {
                switchTasks();
            }
            
            target_shoulder_theta = 1.8;
            target_inside_elbow_theta = 4.38;
            while(!armIsAtTarget(0.25, 0.25))
            {
                armToPreset(1.0, 1.0);
                
                switchTasks();
            }
            
            target_shoulder_theta = 2.0;
            target_inside_elbow_theta = pi*2/3;
            while(!armIsAtTarget(0.25, 0.25))
            {
                armToPreset(1.0, 1.0);
                
                switchTasks();
            }
                        
            armFunction = armUserControl;
        }
        else
        {
            //setIntakeIn();

            target_shoulder_theta = 1.0;
            target_inside_elbow_theta = 2.02;
            while(!armIsAtTarget(0.1, 0.1))
            {
                armToPreset(1.0, 1.0, 1.0);
                
                switchTasks();
            }
            
            setWristOut();
            
            while(fabs(target_wrist_theta - wrist_theta) > 0.1)
            {
                armToPreset(0.7, 1.0, 1.0, 0.99, 0.9, 0.6);
                
                switchTasks();
            }
            
            arm_time = 0;
            while(arm_time < 0.25)
            {
                arm_time += dt;
                armToPreset(0.7, 1.0, 1.0, 0.99, 0.9, 0.6);
                switchTasks();
            }
            
            target_shoulder_theta = 0.78;
            target_inside_elbow_theta = 2.02;
            while(!armIsAtTarget(0.1, 0.1))
            {
                armToPreset(0.7, 1.0, 1.0, 0.99, 0.9, 0.6);
                
                switchTasks();
            }
        }
        
        float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));//+imu_tilt*pi/180.0;
        target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
        
        armFunction = armUserControl;
    }
}

void armPullup()
{
    shoulder = 0;
    winch = 0;
    
    switch(arm_line)
    {
        case 0:default:
        {}
        
        shoulder_active = 0;
        forearm_active = 0;
        
        if(armOnIntakeSide())
        {
            //do nothing
        }
        else
        {
            setIntakeOut();
            
            setWristOut();
            
            target_arm_theta = 1.0;
            target_inside_elbow_theta = pi*7/8;
            while(!armIsAtTarget(0.1, 0.1) || imu_tilt < 45)
            {
                armToPolarTarget(1.0, 1.0);
                
                switchTasks();
            }
            
            target_arm_theta = 1.0;
            target_inside_elbow_theta = pi*3/4;
            while(!armIsAtTarget(0.1, 0.1))
            {
                armToPolarTarget(0.6, 1.0);
                
                switchTasks();
            }
            
            target_arm_theta = 1.5;
            target_inside_elbow_theta = pi*3/4;
            while(!armIsAtTarget(0.1, 0.1))
            {
                armToPolarTarget(0.6, 1.0);
                
                switchTasks();
            }
            
            target_arm_theta = 1.5;
            target_inside_elbow_theta = pi;
            while(!armIsAtTarget(0.1, 0.1))
            {
                armToPolarTarget(0.6, 1.0);
                
                switchTasks();
            }

            target_arm_theta = 1.0;
            target_inside_elbow_theta = pi;
            while(!armIsAtTarget(0.1, 0.1))
            {
                armToPolarTarget(0.6, 1.0);
                
                switchTasks();
            }
        }
        
        float shoulder_axis_to_end = sqrt(sq(forearm_length)+sq(shoulder_length)
                                          -2*forearm_length*shoulder_length*cos(inside_elbow_theta));
        target_arm_theta = shoulder_theta-asin(forearm_length/shoulder_axis_to_end*sin(inside_elbow_theta));
        target_arm_y = shoulder_axis_to_end*cos(target_arm_theta-vertical_arm_theta);
        
        armFunction = armUserControl;
    }
}

#if 0 //arm pathfinding. shelved for now, at least wait until arm simulation is done again
struct arm_state
{
    float shoulder_theta;
    float inside_elbow_theta;
    float intake_theta;
    int hopper_state;
};

bool8 is_valid_arm_position(arm_state s)
{
    
}

enum hopper_state_bits
{
    hand_bit,
    wrist_bit,
    hopper_state_bits
};

#define hand_wrist(hand_on, wrist_on) ((hand_on)<<hand_bit | (wrist_on)<<wrist_bit)

arm_state arm_waypoints[] = {
    {2.00, 4.00, 0.0, hand_wrist(0, 0)},
    {2.20, 4.30, 0.0, hand_wrist(0, 0)},
    {2.35, 4.40, 0.0, hand_wrist(0, 0)},
    {2.46, 4.43, 0.0, hand_wrist(0, 0)},
    {2.46, 4.43, 0.0, hand_wrist(1, 0)},
};

#define triangular_index(x, y) ((((y)*((y)-1))>>1) + (x))

int * waypoint_connections = {
    
    0,
    0,0,
    0,0,0,
    0,0,0,0,
    0,0,0,0,0,
}; //triangular array

void next_waypoint
{
    
}

#undef hand_bit
#undef wrist_bit
#endif

#endif
