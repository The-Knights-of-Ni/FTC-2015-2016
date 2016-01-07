/*
  robot_state_elements
  {
  //76800 = 160*120*4
  //153600 = 160*120*4*2
  //4915200 = 960*1280*4
  //9830400 = 960*1280*4*2
  //TODO: allow for dynamic size
  //TODO: allow for multidimensional arrays
  //byte[9830400] camera_buffers; //alternating buffer, camera_buffers+76800*current_buffer is the current one
  //int current_buffer;
  }
*/

#include "misc.h"
#include "maths.h"
#include <android/native_window.h>
#include <android/native_window_jni.h>

#define camera_w 1280
#define camera_h 960
#define camera_bytes_per_pixel 4

#include "camera_test_robot_state_elements.h"

#define JNI_main Java_com_qualcomm_ftcrobotcontroller_opmodes_CameraTest_main

extern "C"
void JNI_main(JNIEnv * env, jobject self)
{
    initJNI(env, self);

    //TODO: move this to jni_functions?
    jclass cls = env->GetObjectClass(self);
    jfieldID jsurfaceID = env->GetFieldID(cls, "surface", "Landroid/view/Surface;");
    jobject jsurface = env->GetObjectField(self, jsurfaceID);
    
    ANativeWindow * window = ANativeWindow_fromSurface(env, jsurface);
    ANativeWindow_Buffer buffer;

    byte * camera_buffer = 0;
    waitForStart();
    float time = 0;
    do
    {
        {//get camera buffer
                jfieldID jcamera_bufferID = env->GetFieldID(cls, "camera_buffer", "[B");
                jcamera_buffer = (jbyteArray) env->GetObjectField(self, jcamera_buffer);
                
                camera_buffer = (byte *) env->GetByteArrayElements(jcamera_buffer, &camera_buffer.is_copy);
                assert(camera_buffer);
                if(robot_state.is_copy)
                {
                    //TODO: give warning that the GC did not pin the array and perfomance may be impacted
                }
        }
        
        int error = ANativeWindow_lock(window, &buffer, NULL);
        if(error)
        {
            break;
        }

        if(camera_buffer != 0)
        {
            for(int y = 0; y < 100; y++)
            {
                for(int x = 0; x < 100; x++)
                {
                    ((int *) buffer.bits)[x+y*buffer.stride] = (((int*)(camera_buffer+76800*current_buffer))[x+y*camera_w])|0xFF000000;
                    if(x == 99 || y == 99) ((int *) buffer.bits)[x+y*buffer.stride] = 0xFFFFFFFF;
                }
            }
        }
        ANativeWindow_unlockAndPost(window);
    } while(updateRobot(env, self) == 0);
    ANativeWindow_release(window);
    cleanupJNI(env, self);
}
