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
  int camera_w;
  int camera_h;
  }
*/

#include "misc.h"
#include "maths.h"
#include "stdlib.h"

#include <android/native_window.h>
#include <android/native_window_jni.h>

// #define camera_w 1280
// #define camera_h 960
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
    int window_w = ANativeWindow_getWidth(window);
    int window_h = ANativeWindow_getHeight(window);
    ANativeWindow_Buffer buffer;

    byte * camera_buffer = 0;
    jbyteArray jcamera_buffer;
    
    {//get camera buffer
        jfieldID jcamera_bufferID = env->GetFieldID(cls, "camera_buffer", "[B");
        jcamera_buffer = (jbyteArray) env->GetObjectField(self, jcamera_bufferID);
        
        camera_buffer = (byte *) env->GetByteArrayElements(jcamera_buffer, 0);
        assert(camera_buffer);
    }
    
    waitForStart();
    float time = 0;
    do
    {
        
        int error = ANativeWindow_lock(window, &buffer, NULL);
        if(error)
        {
            break;
        }
        
        int r_min = 180;
        int b_min = 160;
        int r_max = 180;
        int g_max = 255;
        int b_max = 180;

        int left_blue = camera_w;
        int right_blue = 0;
        int left_red = camera_w;
        int right_red = 0;

        memset(buffer.bits, 0, window_w*window_h*4);
        for(int y = 0; y < camera_h; y++)
        {
            for(int x = 0; x < camera_w; x++)
            {
                int uv_offset = camera_w*camera_h;
                int y_val = camera_buffer[x+y*camera_w];
                int v_val = camera_buffer[uv_offset+(x&0xFE)+(y>>1)*camera_w];
                int u_val = camera_buffer[uv_offset+1+(x&0xFE)+(y>>1)*camera_w];
                int r = y_val + (1.370705 * (v_val-128));
                int g = y_val - (0.698001 * (v_val-128)) - (0.337633 * (u_val-128));
                int b = y_val + (1.732446 * (u_val-128));
                int color = ((clamp(b, 0, 255)<<16)*0x10000)|((clamp(g, 0, 255)&0xFF)<<8)|((clamp(r, 0, 255)&0xFF));
                if(y_val > 170 && r > r_min && b < b_max && g < g_max)
                {
                    if(x > right_red) right_red = x;
                    if(x < left_red) left_red = x;
                    int draw_x = clamp(x*window_w/camera_w, 0, window_w);
                    int draw_y = clamp(y*window_h/camera_h, 0, window_h);
                    ((int *) buffer.bits)[draw_x+draw_y*buffer.stride] |= 0xFF00FFFF;
                }
                if(y_val > 170 && r < r_max && b > b_min && g < g_max)
                {
                    if(x > right_blue) right_blue = x;
                    if(x < left_blue) left_blue = x;
                    int draw_x = clamp(x*window_w/camera_w, 0, window_w);
                    int draw_y = clamp(y*window_h/camera_h, 0, window_h);
                    ((int *) buffer.bits)[draw_x+draw_y*buffer.stride] |= 0xFFFFFF00;
                }
            }
        }

        for(int y = 0; y < 100; y++)
        {
            int x;
            x = clamp(left_red*window_w/camera_w, 0, window_w);
            ((int *) buffer.bits)[x+y*buffer.stride] = 0xFF0000FF;
            x = clamp(left_blue*window_w/camera_w, 0, window_w);
            ((int *) buffer.bits)[x+y*buffer.stride] = 0xFFFF0000;
        }
        for(int y = 0; y < window_h/2; y++)
        {
            int x;
            x = clamp(right_red*window_w/camera_w, 0, window_w);
            ((int *) buffer.bits)[x+y*buffer.stride] = 0xFF0000FF;
            x = clamp(right_blue*window_w/camera_w, 0, window_w);
            ((int *) buffer.bits)[x+y*buffer.stride] = 0xFFFF0000;
        }
        
        env->ReleaseByteArrayElements(jcamera_buffer, (jbyte *) camera_buffer, JNI_COMMIT);
        ANativeWindow_unlockAndPost(window);
    } while(updateRobot(env, self) == 0);
    ANativeWindow_release(window);
    env->ReleaseByteArrayElements(jcamera_buffer, (jbyte *) camera_buffer, 0);
    cleanupJNI(env, self);
}
