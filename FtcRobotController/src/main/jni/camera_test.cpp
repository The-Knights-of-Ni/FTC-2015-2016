/*
  robot_state_elements
  {
  //76800 = 160*120*4
  byte[76800] camera_buffer; //TODO: allow for dynamic size
  byte[76800] overlay_buffer;
  }
*/

#include "misc.h"
#include "maths.h"

#include "camera_test_robot_state_elements.h"

#define JNI_main Java_com_qualcomm_ftcrobotcontroller_opmodes_CameraTest_main

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);
    
    waitForStart();
    float time = 0;
    do
    {
        
    } while(updateRobot(env, self) == 0);
    cleanupJNI(env, self);
}
