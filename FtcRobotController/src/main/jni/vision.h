#ifndef VISION
#define VISION

#include "jni_functions.h"

int * pindicator;
#define indicator (*pindicator)

int * pbeacon_right;
#define beacon_right (*pbeacon_right)

byte * camera_buffer = 0;
jbyteArray jcamera_buffer;

int camera_w;
int camera_h;

byte * camera_buffer_rgb;

#define camera_bytes_per_pixel 4

void initCamera()
{
    jclass cls = env->GetObjectClass(self);
    
    camera_buffer = 0;
    {//get camera buffer
        jfieldID jcamera_bufferID = env->GetFieldID(cls, "camera_buffer", "[B");
        jcamera_buffer = (jbyteArray) env->GetObjectField(self, jcamera_bufferID);
        
        camera_buffer = (byte *) env->GetByteArrayElements(jcamera_buffer, 0);
        assert(camera_buffer);
    }
    
    jfieldID jcamera_bufferID = env->GetFieldID(cls, "camera_buffer", "[B");
    
    jfieldID camera_wID = env->GetFieldID(cls, "camera_w", "I");
    camera_w = env->GetIntField(cls, camera_wID);
    jfieldID camera_hID = env->GetFieldID(cls, "camera_h", "I");
    camera_h = env->GetIntField(cls, camera_hID);
    camera_buffer_rgb = (byte *) malloc(camera_w*camera_h*camera_bytes_per_pixel);
}

void cleanupCamera()
{
    free(camera_buffer_rgb);
    env->ReleaseByteArrayElements(jcamera_buffer, (jbyte *) camera_buffer, 0);
}

//TODO: simdize
void convertPixelToRGB(int x, int y, int v_val, int u_val)
{
    int y_val = camera_buffer[x+y*camera_w];
    
    int r = y_val + (1.370705 * (v_val-128));
    int g = y_val - (0.698001 * (v_val-128)) - (0.337633 * (u_val-128));
    int b = y_val + (1.732446 * (u_val-128));
    int color = ((clamp(b, 0, 255)<<16)*0x10000)|((clamp(g, 0, 255)&0xFF)<<8)|((clamp(r, 0, 255)&0xFF));
    ((int *) camera_buffer_rgb)[x+y*camera_w] = color;
}

void convertToRGB()
{
    env->ReleaseByteArrayElements(jcamera_buffer, (jbyte *) camera_buffer, JNI_COMMIT);

    int uv_offset = camera_w*camera_h;
    for(int y = 0; y < camera_h; y+=2)
    {
        for(int x = 0; x < camera_w; x+=2)
        {
            int v_val = camera_buffer[uv_offset+(x)+(y>>1)*camera_w];
            int u_val = camera_buffer[uv_offset+1+(x)+(y>>1)*camera_w];
            
            convertPixelToRGB(x  , y  , v_val, u_val);
            convertPixelToRGB(x+1, y  , v_val, u_val);
            convertPixelToRGB(x  , y+1, v_val, u_val);
            convertPixelToRGB(x+1, y+1, v_val, u_val);
        }
    }
}

bool8 getBeaconColor()
{
    convertToRGB();
    
    #define image(x, y, color) camera_buffer[(x+y*camera_w)*camera_bytes_per_pixel+color]
    
    #define reqPixels (camera_w*camera_h/32) //TODO: Make this smarter
    int red_pixel_count[2], blue_pixel_count[2];
    red_pixel_count[0] = 0;
    blue_pixel_count[0] = 0;
    int red_pos[2] = {0, 0};
    int blue_pos[2] = {0, 0};
    for(int i = 0; i < camera_w-5; i+=5)
    {
        red_pixel_count[1] = 0;
        blue_pixel_count[1] = 0;
        for(int j = 0; j < camera_h-5; j+=5)
        {
            if((image(i, j, 0) > 200)){//If at some point there is a pixel that is sufficiently red
                red_pixel_count[1]++;
                red_pixel_count[0]++;
                if(red_pixel_count[0]*red_pixel_count[1] > reqPixels)//If there are enough pixels in this row
                {
                    red_pixel_count[0] = 0;
                    red_pixel_count[1] = 0;
                    red_pos[0] = i;
                    red_pos[1] = j;
                }
            }
            else if(image(i, j, 2) > 200)
            {
                blue_pixel_count[1]++;
                blue_pixel_count[0]++;
                if(blue_pixel_count[0]*blue_pixel_count[1] > reqPixels)//If there are enough pixels in this row
                {
                    blue_pixel_count[0] = 0;
                    blue_pixel_count[1] = 0;
                    blue_pos[0] = i;
                    blue_pos[1] = j;
                }
            }
        }
    }
    //TODO: Add beacon not found case
    #undef image
    
    return red_pos[0] > blue_pos[0];
}
#endif
