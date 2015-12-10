#include "misc.h"
#include <math.h>

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

#endif
