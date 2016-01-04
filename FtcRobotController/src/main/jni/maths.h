#include "misc.h"
#include <math.h>
#include <arm_neon.h>

#ifndef MATHS
#define MATHS

#define pi 3.1415926535897932384626433832795

#define sq(a) ((a)*(a))

#pragma pack(push, 16)
struct v2f
{
    union
    {
        struct
        {
            float x;
            float y;
        };
        float data[2];
    };
    
    inline float & operator[](int a)
    {
        return data[a];
    }
};

struct v3f
{
    union
    {
        struct
        {
            float x;
            float y;
            float z;
        };
        struct
        {
            float i;
            float j;
            float k;
        };
        float data[3];
    };
    
    inline float & operator[](int a)
    {
        return data[a];
    }
};

struct v4f
{
    union
    {
        struct
        {
            float x;
            float y;
            float z;
            float w;
        };
        struct
        { //quaternion notation
            float i;
            float j;
            float k;
            float r;
        };
        float data[4];
    };
    
    inline float & operator[](int a)
    {
        return data[a];
    }
};

struct m3x3f
{
    union
    {
        float data[9];
        struct
        {
            v3f r0;
            v3f r1;
            v3f r2;
        };
        v3f rows[3];
    };

    inline float & operator[](int a)
    {
        return data[a];
    }
};

struct m4x4f
{
    union
    {
        float data[16];
        struct
        {
            v4f r0;
            v4f r1;
            v4f r2;
            v4f r3;
        };
        v4f rows[4];
    };

    inline float & operator[](int a)
    {
        return data[a];
    }
};
#pragma pack(pop)

float bound(float bounded, float lower, float upper)
{
    if(bounded > upper)
        return upper;
    if(bounded < lower)
        return lower;
    return bounded;
}

float lerp(float a, float b, float t)
{
    if(t > 1.0f) return b;
    if(t < 0.0f) return a;
    return a+(b-a)*t;
}

float invSqrt(float a)
{
    int ai = Float.floatToRawIntBits(a);
    int isqrti;
    float isqrt;
    isqrti = 0x5f375a86 - (ai >> 1); //magic
    isqrt = Float.intBitsToFloat(isqrti);
    isqrt *= (1.5f - (a*0.5f*isqrt*isqrt));
    isqrt *= (1.5f - (a*0.5f*isqrt*isqrt)); //second itteration, uncomment if greater accuracy is needed
    return isqrt;
}
//TODO: Fix this
void normalizeScale(float* v, float s)
{
    float normsq = v[0]*v[0]+v[1]*v[1];
    scale(v, invSqrt(normsq)*s);
}


//TODO: Vector Add (2D and 3D)

//TODO: Vector Subtract (2D and 3D)
//TODO: Norm
//TODO: Normalize
//TODO: Cross
//TODO: Dot
//TODO: Project





















#endif
