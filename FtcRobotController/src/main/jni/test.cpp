/*
  robot_state_elements
  {
  gamepad{float joystick_1_x, float joystick_1_y};
  
  float left_drive_power;
  float right_drive_power;
  gamepad gamepad1;
  byte[100] array_example;
  
  //general syntax
  //type{primitive_type name in java, ...};
  //type name;
  }
*/

#include "misc.h"
#include "maths.h"

#include <jni.h>

struct gamepad
{
    v2f left_stick;
};

#include "test_robot_state_elements.h"

#define JNI_main Java_com_qualcomm_ftcrobotcontroller_opmodes_NDK_1test_main

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    float time = 0;
    for ever
    {
        //(*((float*)(robot_state.state+rsid_left_drive_power))) = 1.0f;//robot_state.state[rsid_gamepad1];
        //left_drive_power = 5.206;
        //right_drive_power = 5206.0;
        left_drive_power = gamepad1.left_stick.y-gamepad1.left_stick.x;
        right_drive_power = gamepad1.left_stick.y+gamepad1.left_stick.x;
        updateRobot(env, self);
    }
    cleanupJNI(env, self);
}
