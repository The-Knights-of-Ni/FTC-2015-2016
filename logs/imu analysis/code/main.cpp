#include <string.h>
#include <stdio.h>
#include <math.h>
#include "misc.h"

#define STBIW_ASSERT(x) assert(x)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define constStrncmp(a, constant_string, max_len) (strncmp((a), constant_string, min(max_len, sizeof(constant_string)-1)))

struct data_point
{
    float time;
    int x;
    int y;
    int z;
};

data_point * data;
uint n_data_points = 0;

struct filter_out
{
    float x;
    float v;
};

inline float32 fast_cos(float32 x)
{
    x = fabs(x);
    x -= floor(x);
    x -= 0.5;
    x = fabs(x);
    x -= 0.25;
    //x is converted to a triangle wave since the polynomial approxomation only works between -pi/4 and pi/4 radians
    float32 out = -32.0;
    out *= x;
    out *= x;
    out += 6.0;
    out *= x;
    return out;//x*(6.0-32.0*x*x);
}

inline float32 fast_sin(float32 x)
{
    return cos(x - 0.25);
}

const float k = 0.5;
const float c = 1.0;
const float m = 1.0;

float lambda = -c/(2.0*m);
float omega = sqrt(4.0*k/m-sq(c/m));

filter_out filter(float x_1, float x_0, float v_0, float dt)
{
    float c_1 = x_0-x_1;
    float c_2 = v_0/(omega*lambda);

    filter_out out;
    //small angle approx for sin and cos
    out.x = x_1+exp(lambda*dt)*(c_1*cos(omega*dt) + c_2*sin(omega*dt));
    out.v = omega*lambda*exp(lambda*dt)*(-c_1*sin(omega*dt) + c_2*cos(omega*dt));
    return out;
}

void drawLine(int x0, int y0, int x1, int y1, byte color, byte * bitmap, int bitmap_width, int bitmap_height)
{    
    if(x0 < 0)
    {
        float t = fabs(((float)x0-0)/((float)x1-x0));
        x0 = 0;
        y0 = lerp(y0, y1, t);
    }
    if(x0 >= bitmap_width)
    {
        float t = fabs(((float)x0-bitmap_width+1)/((float)x1-x0));
        x0 = bitmap_width-1;
        y0 = lerp(y0, y1, t);
    }
    if(x1 < 0)
    {
        float t = fabs(((float)x1-0)/((float)x0-x1));
        x1 = 0;
        y1 = lerp(y1, y0, t);
    }
    if(x1 >= bitmap_width)
    {
        float t = fabs(((float)x1-bitmap_width+1)/((float)x0-x1));
        x1 = bitmap_width-1;
        y1 = lerp(y1, y0, t);
    }

    if(y0 < 0)
    {
        if(y1 < 0) return;
        float t = fabs(((float)y0-0)/((float)y1-y0));
        y0 = 0;
        x0 = lerp(x0, x1, t);
    }
    if(y0 >= bitmap_height)
    {
        if(y1 >= bitmap_height) return;
        float t = fabs(((float)y0-bitmap_height+1)/((float)y1-y0));
        y0 = bitmap_height-1;
        x0 = lerp(x0, x1, t);
    }
    if(y1 < 0)
    {
        if(y0 < 0) return;
        float t = fabs(((float)y1-0)/((float)y0-y1));
        y1 = 0;
        x1 = lerp(x1, x0, t);
    }
    if(y1 >= bitmap_height)
    {
        if(y0 >= bitmap_height) return;
        float t = fabs(((float)y1-bitmap_height+1)/((float)y0-y1));
        y1 = bitmap_height-1;
        x1 = lerp(x1, x0, t);
    }
    if(x0 == x1 && y0 == y1) return;
    int n_points = max(absi(x0-x1), absi(y0-y1));
    if(n_points > 0)
    {
        for(int t = 0; t <= n_points; t++)
        {
            int x = interp(x0, x1, t, n_points);
            int y = interp(y0, y1, t, n_points);
            bitmap[(x+y*bitmap_width)*3+color] = 0xFF;
        }
    }
}

void getData(char * in, uint in_len)
{
    float time;
    int i = 0;
    while(i < in_len)
    {
        if(constStrncmp(in+i, "raw: ", in_len-i) == 0)
        {
            i += sizeof("raw:");
            sscanf(in+i, "%d, %d, %d", &data[n_data_points].x, &data[n_data_points].y, &data[n_data_points].z);
            data[n_data_points].time = time;
            //printf("%f, %d, %d, %d\n", data[n_data_points].time, data[n_data_points].x, data[n_data_points].y, data[n_data_points].z);
            n_data_points++;
        }
        if(constStrncmp(in+i, "dt: ", in_len-i) == 0)
        {
            i += sizeof("dt:");
            float dt;
            sscanf(in+i, "%f", &dt);
            time += dt;
        }
        for(; i < in_len && in[i] != '\n'; i++){}
        i++;
    }
}

int main()
{
    FILE * log_file = fopen("imu_test_log3.txt", "r");
    assert(log_file);
    
    int error = fseek(log_file, 0, SEEK_END);
    assert(error == 0);
    
    size_t log_file_size = ftell(log_file);
    assert(log_file_size != -1);
    
    error = fseek(log_file, 0, SEEK_SET);
    assert(error == 0);
    
    char * log = (char *) malloc(log_file_size+1);
    
    fread(log, sizeof(char), log_file_size, log_file);
    log[log_file_size] = 0;
    fclose(log_file);
    
    void * free_memory = malloc(2000000);
    assert(free_memory);
    
    data = (data_point *) free_memory;
    getData(log, log_file_size);
    free_memory = (void*)((data_point *) free_memory+n_data_points);
    
    int bitmap_width = 4*n_data_points;
    int bitmap_height = 1024;
    byte * bitmap = (byte*) malloc(bitmap_width*bitmap_height*3);
    assert(bitmap);
    memset(bitmap, 0x00, bitmap_width*bitmap_height*3);
    int b = 0;
    
    int max_data_y = 200;
    
    int y0r = bitmap_height/2-(bitmap_height*data[0].x)/max_data_y;
    int y0g = bitmap_height/2-(bitmap_height*data[0].y)/max_data_y;
    int y0b = bitmap_height/2-(bitmap_height*data[0].z)/max_data_y;
    int x0 = data[0].time;


    int hpy0r = bitmap_height/2-(bitmap_height*data[0].x)/max_data_y;
    int hpy0g = bitmap_height/2-(bitmap_height*data[0].y)/max_data_y;
    int hpy0b = bitmap_height/2-(bitmap_height*data[0].z)/max_data_y;
    float hpyr = 0.0;
    float hpyg = 0.0;
    float hpyb = 0.0;
    float hpyrv = 0.0;
    float hpygv = 0.0;
    float hpybv = 0.0;
    
    for(int i = 1; i < n_data_points; i++)
    {
        printf("graphing: %d%%\n", (100*i)/(n_data_points-1));        
        int x1 = (bitmap_width*data[i].time)/data[n_data_points-1].time+0.5;
        int y1;
        
        y1 = bitmap_height/2-(bitmap_height*data[i].x)/max_data_y;
        drawLine(x0, y0r, x1, y1, 0, bitmap, bitmap_width, bitmap_height);
        y0r = y1;
        
        y1 = bitmap_height/2-(bitmap_height*data[i].y)/max_data_y;
        drawLine(x0, y0g, x1, y1, 1, bitmap, bitmap_width, bitmap_height);
        y0g = y1;
        
        y1 = bitmap_height/2-(bitmap_height*data[i].z)/max_data_y;
        drawLine(x0, y0b, x1, y1, 2, bitmap, bitmap_width, bitmap_height);
        y0b = y1;
        
        // filter_out r_out = filter((float) data[i].x, hpyr, hpyrv, data[i].time-data[i-1].time);
        // hpyr = r_out.x;
        // hpyrv = r_out.v;
        
        // filter_out g_out = filter((float) data[i].y, hpyg, hpygv, data[i].time-data[i-1].time);
        // hpyg = g_out.x;
        // hpygv = g_out.v;
        
        // filter_out b_out = filter((float) data[i].z, hpyb, hpybv, data[i].time-data[i-1].time);
        // hpyb = b_out.x;
        // hpybv = b_out.v;
        
        // int hpy1;
        
        // hpy1 = bitmap_height/2-(bitmap_height*hpyr)/max_data_y;
        // drawLine(x0, hpy0r, x1, hpy1, 0, bitmap, bitmap_width, bitmap_height);
        // hpy0r = hpy1;

        // hpy1 = bitmap_height/2-(bitmap_height*hpyg)/max_data_y;
        // drawLine(x0, hpy0g, x1, hpy1, 1, bitmap, bitmap_width, bitmap_height);
        // hpy0g = hpy1;
        
        // hpy1 = bitmap_height/2-(bitmap_height*hpyb)/max_data_y;
        // drawLine(x0, hpy0b, x1, hpy1, 2, bitmap, bitmap_width, bitmap_height);
        // hpy0b = hpy1;
        
        x0 = x1;
    }
    // for(int x = 10; x < 20; x++)
    // {
    //     for(int y = 10; y < 20; y++)
    //     {
    //         bitmap[(x+y*bitmap_width)*3] = 0xFF;
    //         bitmap[(x+y*bitmap_width)*3+1] = 0xFF;
    //         bitmap[(x+y*bitmap_width)*3+2] = 0xFF;
    //     }
    // }
    stbi_write_png("graph3.png", bitmap_width, bitmap_height, 3, bitmap, bitmap_width*3);
    
    return 0;
}
