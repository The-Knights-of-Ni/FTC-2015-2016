//This is not a comment
/*
  robot_state_elements
  {

//  int right_drive_encoder;
//  int left_drive_encoder;
//  int elbow_encoder;
//  int shoulder_encoder;
//  int potentiometer;
//  float heading;
//  float tilt;
//  float roll;
//  float x_velocity;
//  float y_velocity;
//
//  float left_drive;
//  float right_drive;
//  float elbow;
//  float shoulder;
//  float intake;
//  float hand;
//  float slide;

  int indicator;
  //general syntax
  //type{primitive_type name in java, ...};
  //type name;
  }
*/

#include "drive.h"
#include "arm.h"

#include "Mk3Auto_robot_state_elements.h"

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

    //Config
    //hopper down
    intake = 1;
    if(updateRobot() != 0) exit(EXIT_SUCCESS);
    /*turnBot(45, 0.8);
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

    cleanupJNI();
}