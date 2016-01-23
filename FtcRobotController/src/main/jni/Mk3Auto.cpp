//This is not a comment
/*
  robot_state_elements
  {
  //TODO: if we stick with the robot_state_elements thing add support for typedefs
  //(c type with a different name and element names than the robot state element type)
  
  v3f{float x, float y, float z}; //TODO; support space after struct name
  //v4f{float i, float j, float k, float r}; //for quaternions
  
  double time;
  
  int right_drive_encoder;
  int left_drive_encoder;
  int winch_encoder;
  int shoulder_encoder;
  int elbow_potentiometer;
  float imu_heading;
  float imu_tilt;
  float imu_roll;
  v3f imu_velocity;
  int color;
  
  float left_drive;
  float right_drive;
  float winch;
  float shoulder;
  float intake;
  float hand;
  float slide;
  
  int indicator;

  int camera_w;
  int camera_h;

  int beacon_right;
  //general syntax
  //type{primitive_type name in java, ...};
  //type name;
  }
*/

void customAutonomousUpdate();
#include "autonomous.h" //NOTE: needs customAutonomousUpdate must be declared before including this
#include "vision.h"

#include "Mk3Auto_robot_state_elements.h"

//stuff that need to be constantly updated in the background but is not intensive enough to deserve a seperate thread
void customAutonomousUpdate()
{
    dt = time-current_time;
    current_time = time;
    
    //TODO: make this sensor filter stuff a function in arm.h
    elbow_potentiometer_angle = lerp(
        (360-((180.0f-potentiometer_range*0.5f+potentiometer_range*(elbow_potentiometer/(1023.0f)))+12.0f))*pi/180.0f,
        elbow_potentiometer_angle,
        exp(-500.0*dt));
    
    float new_shoulder_theta = shoulder_encoder/shoulder_gear_ratio/encoderticks_per_radian+pi*150/180.0;
    float new_inside_elbow_theta = elbow_potentiometer_angle;
    float new_winch_theta = winch_encoder/winch_gear_ratio/encoderticks_per_radian;
    shoulder_omega = lerp((new_shoulder_theta-shoulder_theta)/dt, shoulder_omega, 0.1);
    winch_omega = lerp((new_winch_theta-winch_theta)/dt, winch_omega, 0.1);
    float inside_elbow_omega = (new_inside_elbow_theta-inside_elbow_theta)/dt;
    
    shoulder_theta = new_shoulder_theta;
    inside_elbow_theta = new_inside_elbow_theta;
    winch_theta = new_winch_theta;

    shoulder = 0;
    winch = 0;
    armJointsToAngle(shoulder, winch,
               target_shoulder_theta, target_inside_elbow_theta,
               shoulder_theta, inside_elbow_theta,
               score_mode, dt);
    
    shoulder = clamp(shoulder, -1.0, 1.0);
    winch = clamp(winch, -1.0, 1.0);
    left_drive = clamp(left_drive, -1.0, 1.0);
    right_drive = clamp(right_drive, -1.0, 1.0);
    intake = clamp(intake, -1.0, 1.0);
    
    hand = clamp(hand, 0.0, 1.0);
    slide = clamp(slide, 0.0, 1.0);
}

#define JNI_main Java_com_qualcomm_ftcrobotcontroller_opmodes_Mk3Auto_main

//#define update (if(updateRobot() == 0) exit(EXIT_SUCCESS););
#define currentColor 0
#define visionColor 0 //Right side of the beacon
#define slide_position_right 50
#define slide_position_left -70
#define slide_position_safe 90

extern "C"
void JNI_main(JNIEnv * _env, jobject _self)
{
    //NOTE: DON'T FORGET THESE //TODO: make is so you don't need these
    env = _env;
    self = _self;
    
    initJNI();
    waitForStart();
    initCamera(camera_w, camera_h);
    
    setDriveMotors(&left_drive, &right_drive, &imu_heading, &left_drive_encoder, &right_drive_encoder);
    
    //waitForStart(); //needs to be called in java until IMU code is ported
    
    current_time = 0;
    //Config
    //hopper down
    #define colorAdjustedAngle(a) (currentColor ? a : -a)
    
    interruptable
    {
        // for ever
        // {
        //     beacon_right = getBeaconColor();
        //     updateRobot();
        // }
        
        target_shoulder_theta = pi*140/180;
        target_inside_elbow_theta = pi*210/180;
        for ever
        {
            autonomousUpdate();
        }
        wait(1);
        intake = 1;
        autonomousUpdate();
        //driveOnCourseIn(-10, 0.8, 45);
        turnRelDeg(45, 0.8);
        #if 0
        driveOnCourseIn(-80, 0.8, 45);
        intake = 0;
        (colorAdjustedAngle(45), 0.8);
        //vision
        if (visionColor == currentColor)
        {
            slide = slide_position_right;
        }
        else
            slide = slide_position_left;
        wait(1);
        driveDistIn(-5, 0.4);
        //score climbers, ideally without turning
        target_arm_theta = 150;
        target_inside_elbow_theta = 180;
        wait(1);
        driveDistIn(24, 0.8);
        turnRelDeg(colorAdjustedAngle(45), 0.8);
        driveDistIn(24);
        turnRelDeg(colorAdjustedAngle(90), 0.8);
        //driveDistIn(60);
        #endif
        //drive to nearest mountain, park low
    }
}
