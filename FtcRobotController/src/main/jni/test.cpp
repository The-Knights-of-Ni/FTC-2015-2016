/*
  robot_state_elements
  {
  gamepad{float joystick_1_x, float joystick_2_y};
  
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

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    float time = 0;
    for ever
    {
        left_drive_power = gamepad1.left_stick.y-gamepad1.left_stick.x;
        right_drive_power = gamepad1.left_stick.y+gamepad1.left_stick.x;
        updateRobot(env, self);
    }
    cleanupJNI(env, self);
}
