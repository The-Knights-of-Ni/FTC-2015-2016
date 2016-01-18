/*
  robot_state_elements
  {
  byte[100] camera_buffer; //TODO: figure out this size
  }
*/

#include "misc.h"
#include "maths.h"

#include "camera_test_robot_state_elements.h"

#define JNI_main Java_com_qualcomm_ftcrobotcontroller_opmodes_CameraTest_main

extern "C"
void JNI_main(JNIEnv * _env, jobject _self)
{
    env = _env;
    self = _self;
    
    initJNI();
    
    waitForStart();
    float time = 0;
    for ever
    {
        updateRobot();
    }
    cleanupJNI();
}
