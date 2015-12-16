#include <string.h>
#include <stdio.h>
#include <math.h>
#include "misc.h"

#define STBIW_ASSERT(x) assert(x)
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

#define constStrncmp(a, constant_string, max_len) (strncmp((a), constant_string, min(max_len, sizeof(constant_string)-1)))

//from <http://aggregate.ee.engr.uky.edu/MAGIC/#Log2 of an Integer>
uint ones32(register uint x)
{
        /* 32-bit recursive reduction using SWAR...
	   but first step is mapping 2-bit values
	   into sum of 2 1-bit values in sneaky way
	*/
        x -= ((x >> 1) & 0x55555555);
        x = (((x >> 2) & 0x33333333) + (x & 0x33333333));
        x = (((x >> 4) + x) & 0x0f0f0f0f);
        x += (x >> 8);
        x += (x >> 16);
        return(x & 0x0000003f);
}

uint flog2(register uint x)
{
    x |= (x >> 1);
    x |= (x >> 2);
    x |= (x >> 4);
    x |= (x >> 8);
    x |= (x >> 16);
    return ones32(x>>1);
}

//NOTE: the input is not preserved
void haarWaveletTransform(int * in, int * out, int n)
{
    for(int length = (1<<n)>>1; ; length>>=1)
    {
        for(int i = 0; i < length; i++)
        {
            out[i] = (in[2*i]+in[2*i+1])>>1;
            out[i+length] = (in[2*i]-in[2*i+1])>>1;
            if(absi(out[i]) < 30) out[i] = 0;
            if(absi(out[i+length]) < 30) out[i+length] = 0;
        }
        //return;//TEMP
        if(length == 1)
        {
            return;
        }
        memcpy(in, out, sizeof(int)*(length<<1));
    }
}

void inverseHaarWaveletTransform(int * in, int * out, int n)
{
    for(int length = 1; ; length<<=1)
    {
        //length = (1<<n)>>1;//TEMP
        for(int i = 0; i < length; i++)
        {
            out[2*i] = in[i]+in[i+length];
            out[2*i+1] = in[i]-in[i+length];
        }
        //return;//TEMP
        if(length == (1<<n)>>1)
        {
            return;
        }
        memcpy(in, out, sizeof(int)*(length<<1));
    }
}

struct data_point
{
    float time;
    int x;
    int y;
    int z;
};

float * data_time;
int * data_x;
int * data_y;
int * data_z;
uint n_data_points = 0;

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
            sscanf(in+i, "%d, %d, %d", &data_x[n_data_points], &data_y[n_data_points], &data_z[n_data_points]);
            data_time[n_data_points] = time;
            //printf("%f, %d, %d, %d\n", data_time[n_data_points], data_x[n_data_points], data_y[n_data_points], data_z[n_data_points]);
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
        
    data_time = (float *) free_memory;
    free_memory = (void*)((float *) free_memory+50000);
    
    data_x = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+50000);
    
    data_y = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+50000);
    
    data_z = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+50000);
    getData(log, log_file_size);
    printf("n_data_points: %d\n", n_data_points);
    
    int transform_n = flog2(n_data_points);
    
    printf("about to transform\n");
    
    //x
    int * transformed_x_in = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+(1<<transform_n));
    memcpy(transformed_x_in, data_x, sizeof(int)*(1<<transform_n));
    
    int * transformed_x = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+(1<<transform_n));
    
    haarWaveletTransform(transformed_x_in, transformed_x, transform_n);

    //y
    int * transformed_y_in = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+(1<<transform_n));
    memcpy(transformed_y_in, data_y, sizeof(int)*(1<<transform_n));
    
    int * transformed_y = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+(1<<transform_n));
    
    haarWaveletTransform(transformed_y_in, transformed_y, transform_n);

    //z
    int * transformed_z_in = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+(1<<transform_n));
    memcpy(transformed_z_in, data_z, sizeof(int)*(1<<transform_n));
    
    int * transformed_z = (int *) free_memory;
    free_memory = (void*)((int *) free_memory+(1<<transform_n));
    
    haarWaveletTransform(transformed_z_in, transformed_z, transform_n);
    
    printf("about to inverse transform\n");
    
    //inverse transform
    inverseHaarWaveletTransform(transformed_x, transformed_x_in, transform_n);
    inverseHaarWaveletTransform(transformed_y, transformed_y_in, transform_n);
    inverseHaarWaveletTransform(transformed_z, transformed_z_in, transform_n);
    
    int bitmap_width = 4*n_data_points;
    int bitmap_height = 1024;
    byte * bitmap = (byte*) malloc(bitmap_width*bitmap_height*3);
    assert(bitmap);
    memset(bitmap, 0x00, bitmap_width*bitmap_height*3);
    int b = 0;
    
    int max_data_y = 300;
    
    int y0r = bitmap_height/2-(bitmap_height*data_x[0])/max_data_y;
    int y0g = bitmap_height/2-(bitmap_height*data_y[0])/max_data_y;
    int y0b = bitmap_height/2-(bitmap_height*data_z[0])/max_data_y;
    int x0 = data_time[0];
    
    
    int fy0r = bitmap_height/2-(bitmap_height*data_x[0])/max_data_y;
    int fy0g = bitmap_height/2-(bitmap_height*data_y[0])/max_data_y;
    int fy0b = bitmap_height/2-(bitmap_height*data_z[0])/max_data_y;
    
    float fvx = 0.0;
    
    int fv0r = 0;
    int fv0g = 0;
    int fv0b = 0;
    
    for(int i = 1; i < n_data_points; i++)
    {
        printf("graphing: %d%%\n", (100*i)/(n_data_points-1));
        int x1 = (bitmap_width*data_time[i])/data_time[n_data_points-1];
        int y1;
        
        y1 = bitmap_height/2-(bitmap_height*data_x[i])/max_data_y;
        drawLine(x0, y0r, x1, y1, 0, bitmap, bitmap_width, bitmap_height);
        y0r = y1;
        
        // y1 = bitmap_height/2-(bitmap_height*data_y[i])/max_data_y;
        // drawLine(x0, y0g, x1, y1, 1, bitmap, bitmap_width, bitmap_height);
        // y0g = y1;
        
        // y1 = bitmap_height/2-(bitmap_height*data_z[i])/max_data_y;
        // drawLine(x0, y0b, x1, y1, 2, bitmap, bitmap_width, bitmap_height);
        // y0b = y1;
        
        if(i < (1<<transform_n))
        {
            int fy1;
            fy1 = bitmap_height/2-(bitmap_height*transformed_x_in[i])/max_data_y/2;
            drawLine(x0, fy0r, x1, fy1, 2, bitmap, bitmap_width, bitmap_height);
            fy0r = fy1;
            
            int fv1;
            fvx += transformed_x_in[i];
            //if(fvx < 1000 && transformed_x_in[i] < 20) fvx = 0;
            fv1 = bitmap_height/2-(bitmap_height*fvx)/1000;
            drawLine(x0, fv0r, x1, fv1, 1, bitmap, bitmap_width, bitmap_height);
            fv0r = fv1;
            
            // fy1 = bitmap_height/2-(bitmap_height*transformed_y_in[i])/max_data_y;
            // drawLine(x0, fy0g, x1, fy1, 1, bitmap, bitmap_width, bitmap_height);
            // fy0g = fy1;
            
            // fy1 = bitmap_height/2-(bitmap_height*transformed_z_in[i])/max_data_y;
            // drawLine(x0, fy0b, x1, fy1, 2, bitmap, bitmap_width, bitmap_height);
            // fy0b = fy1;
        }
        
        x0 = x1;
    }
    
    printf("drawing x-axis\n");
    for(int i = 0; i < bitmap_width; (i%10 >= 5) ? i+=5 : i++)
    {
        bitmap[(i+(bitmap_height/2*bitmap_width))*3+0] = 0xFF;
        bitmap[(i+(bitmap_height/2*bitmap_width))*3+1] = 0xFF;
        bitmap[(i+(bitmap_height/2*bitmap_width))*3+2] = 0xFF;
    }
    printf("done drawing x-axis\n");
    
    stbi_write_png("graph3 with wavelet filter.png", bitmap_width, bitmap_height, 3, bitmap, bitmap_width*3);
    
    return 0;
}
