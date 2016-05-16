 #ifndef VISION
#define VISION

#include "jni_functions.h"

int * pindicator;
#define indicator (*pindicator)

int * pbeacon_right;
#define beacon_right (*pbeacon_right)

byte * camera_buffer = 0;
jbyteArray jcamera_buffer;

int camera_w = 0;
int camera_h = 0;

byte * camera_buffer_rgb;

#define camera_bytes_per_pixel 4

void initCamera()
{
    //jclass cls = env->GetObjectClass(self);
    
    camera_buffer = 0;
    {//get camera buffer
        jfieldID jcamera_bufferID = env->GetFieldID(cls, "camera_buffer", "[B");
        jcamera_buffer = (jbyteArray) env->GetObjectField(self, jcamera_bufferID);
        
        camera_buffer = (byte *) env->GetByteArrayElements(jcamera_buffer, 0);
        assert(camera_buffer);
    }
    
    jfieldID camera_wID = env->GetFieldID(cls, "camera_w", "I");
    camera_w = env->GetIntField(self, camera_wID);
    jfieldID camera_hID = env->GetFieldID(cls, "camera_h", "I");
    camera_h = env->GetIntField(self, camera_hID);
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

struct HSL
{
    float data[3];
    inline float &operator[](int a)
    {
        return data[a];
    }
};

inline HSL getHue(int R, int G, int B) //TODO: SIMD
{
    HSL hsl;
    float R_frac = R/255.0;
    float G_frac = G/255.0;
    float B_frac = B/255.0;
    float max_val = (R_frac >= G_frac ? (R_frac >= B_frac ? R_frac : B_frac) : (G_frac >= B_frac ? G_frac : B_frac));
    float min_val = (R_frac <= G_frac ? (R_frac <= B_frac ? R_frac : B_frac) : (G_frac <= B_frac ? G_frac : B_frac));
    //Luminance
    hsl[2] = (max_val + min_val)/2.0;
    //Saturation
    if(hsl[2] < 0.5)
    hsl[1] = (max_val - min_val)/(max_val+min_val);
    else
    hsl[1] = (max_val - min_val)/(2.0 - max_val - min_val);
    //Hue
    if(R_frac == max_val)
    hsl[0] = (G_frac - B_frac)/(max_val - min_val);
    else if(G_frac == max_val)
    hsl[0] = 2.0 + (B_frac - R_frac)/(max_val - min_val);
    else
    hsl[0] = 4.0 + (R_frac - G_frac)/(max_val - min_val);
    //Processing
    hsl[0] *= 60;
    if(hsl[0] < 0)
    hsl[0] += 360;
    hsl[1] *= 100;
    hsl[2] *= 100;
    return hsl;
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
    
    #define reqPixelz (camera_w*camera_h/512) //TODO: Make this smarter
    int reqPixels = reqPixelz;
    int red_pixel_count[2] = {0, 0};
    int blue_pixel_count[2] = {0, 0};
    int red_pos[2] = {0, 0};
    int blue_pos[2] = {0, 0};
    HSL value;
    for(int i = 0; i < camera_w-5; i+=5)
    {
        red_pixel_count[1] = 0;
        blue_pixel_count[1] = 0;
        for(int j = 0; j < camera_h-5; j+=5)
        {
            value = getHue(image(i, j, 0), image(i, j, 1), image(i, j, 2));
            int hue = value[0];
            int saturation = value[1];
            int light = value[2];
            if((hue > 300 || hue < 45) && ((saturation + light) > 140)){//If at some point there is a pixel that is sufficiently red and not blue
                red_pixel_count[1]++;
                red_pixel_count[0]++;
                log("r pixel count %d ", red_pixel_count[0]*red_pixel_count[1]);
                //highlight.draw_circle(i,j,5,red,1.0f).display(main_disp);
                if(red_pixel_count[0]*red_pixel_count[1] > reqPixels)//If there are enough pixels in this row
                {
                    //printf("Red Flag has reached high enough value @ %i, %i, %i, %i\n", i, j, red_pixel_count[0], red_pixel_count[1]);
                    red_pixel_count[0] = 0;
                    red_pixel_count[1] = 0;
                    red_pos[0] = i;
                    red_pos[1] = j;
                    log("r");
                }
            }
            else if((hue > 170 && hue < 200) && ((saturation + light) > 140))
            {
                blue_pixel_count[1]++;
                blue_pixel_count[0]++;
                log("b pixel count %d ", blue_pixel_count[0]*blue_pixel_count[1]);
                //highlight.draw_circle(i,j,5,blue,1.0f).display(main_disp);
                if(blue_pixel_count[0]*blue_pixel_count[1] > reqPixels)//If there are enough pixels in this row TODO: add y dim to this for more accuracy
                {
                    //					printf("Blue Flag has reached high enough value @ %i, %i, %i, %i\n", i, j, blue_pixel_count[0], blue_pixel_count[1]);
                    blue_pixel_count[0] = 0;
                    blue_pixel_count[1] = 0;
                    blue_pos[0] = i;
                    blue_pos[1] = j;
                    log("b");
                }
            }
            else
            {
                // const unsigned char colorz[] = {image(i,j,0), image(i,j,1), image(i,j,2)};
                //				highlight.draw_point(i, j, colorz,1.0f);//There has to be a better way than this
            }
        }
    }
    //TODO: Add beacon not found case
    #undef image

    log("%d, %d, ", red_pos[0], blue_pos[0]);
    
    return red_pos[0] > blue_pos[0];
}
#endif
