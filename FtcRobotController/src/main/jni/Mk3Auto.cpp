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
  
  float left_drive;
  float right_drive;
  float winch;
  float shoulder;
  float intake;
  float hand;
  float slide;
  
  int indicator;
  //general syntax
  //type{primitive_type name in java, ...};
  //type name;
  }
*/

void customAutonomousUpdate();
#include "autonomous.h" //NOTE: needs customAutonomousUpdate must be declared before including this

#include "Mk3Auto_robot_state_elements.h"

float old_time;

//stuff that need to be constantly updated in the background but is not intensive enough to deserve a seperate thread
void customAutonomousUpdate()
{
    float dt = time-old_time;
    old_time = time;
    
    //TODO: make this sensor filter stuff a function in arm.h
    elbow_potentiometer_angle = lerp(
        (360-((180.0f-potentiometer_range*0.5f+potentiometer_range*(elbow_potentiometer/(1023.0f)))+12.0f))*pi/180.0f,
        elbow_potentiometer_angle,
        exp(-20.0*dt));
    
    float new_shoulder_theta = shoulder_encoder/shoulder_gear_ratio/encoderticks_per_radian+pi*150/180.0;
    float new_inside_elbow_theta = elbow_potentiometer_angle;
    float new_winch_theta = winch_encoder/winch_gear_ratio/encoderticks_per_radian;
    shoulder_omega = lerp((new_shoulder_theta-shoulder_theta)/dt, shoulder_omega, 0.1);
    winch_omega = lerp((new_winch_theta-winch_theta)/dt, winch_omega, 0.1);
    float inside_elbow_omega = (new_inside_elbow_theta-inside_elbow_theta)/dt;
    
    shoulder_theta = new_shoulder_theta;
    inside_elbow_theta = new_inside_elbow_theta;
    winch_theta = new_winch_theta;
    
    armToAngle(shoulder, winch,
               target_arm_theta, target_inside_elbow_theta,
               shoulder_theta, inside_elbow_theta,
               score_mode, dt);
}

#define JNI_main Java_com_qualcomm_ftcrobotcontroller_opmodes_Mk3Auto_main
extern "C"

//#define update (if(updateRobot() == 0) exit(EXIT_SUCCESS););
#define currentColor 0
#define visionColor 0 //Right side of the beacon
#define slide_position_right 50
#define slide_position_left -70
#define slide_position_safe 90

void JNI_main(JNIEnv * env, jobject self)
{
    initJNI();
    
    waitForStart();

    old_time = 0;
    //Config
    //hopper down
    interruptable for ever
    {
        intake = 1;
        autonomousUpdate();
        turnRelDeg(left_drive, right_drive, 45, 0.8, &imu_heading);
        /*
          driveOnCourseIn(80, 0.8, 45);
          intake = 0;
          update
          turnBot(45, 0.8);
          //vision
          if (visionColor == currentColor)
          {
          slide = slide_position_right;
          }
          else
          slide = slide_position_left;
          update
          driveDistIn(5, 0.4);
          //score climbers, ideally without turning
          //drive to nearest mountain, park low
          */
    }
}
