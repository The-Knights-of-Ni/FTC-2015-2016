#include "misc.h"
#include <math.h>
#include <arm_neon.h>

#ifndef MATHS
#define MATHS

#define pi 3.1415926535897932384626433832795
#define e  2.71828182845904523536
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

    inline float &operator[](int a)
    {
        return data[a];
    }

};

/* Start of 2D Vector Functions */
v2f operator+(v2f a, v2f b)
{
    v2f output;
    output.x = a.x + b.x;
    output.y = a.y + b.y;
    return output;
}

v2f operator-(v2f a, v2f b)
{
    v2f output;
    output.x = a.x - b.x;
    output.y = a.y - b.y;
    return output;
}

v2f operator*(v2f a, float b)
{
    v2f output;
    output.x = a.x * b;
    output.y = a.y * b;
    return output;
}

v2f operator*(float b, v2f a)
{
    v2f output;
    output.x = a.x * b;
    output.y = a.y * b;
    return output;
}

v2f operator/(v2f a, float b)//Shouldn't need other configuration, since it makes no sense
{
    v2f output;
    output.x = a.x / b;
    output.y = a.y / b;
    return output;
}

float dot(v2f a, v2f b)
{
    return a.x * b.x + a.y * b.y;
}

float norm(v2f a)
{
    return sqrt(sq(a.x) + sq(a.y));
}

float normSq(v2f a)
{
    return sq(a.x) + sq(a.y);
}

v2f normalize(v2f a)
{
    v2f output;
    output.x = a.x / norm(a);
    output.y = a.y / norm(a);
    return output;
}

v2f complexMultiply(v2f a, v2f b)
{
    v2f output;
    output.x = a.x * b.x - a.y * b.y;
    output.y = a.x * b.y + a.y * b.x;
    return output;
}

v2f project(v2f a, v2f b)
{
    return dot(a, b) / b.normSq() * b;
}

v2f reject(v2f a, v2f b)
{
    return a - project(a, b);
}

v2f normalizeScale(v2f a, float s)
{
    v2f output;
    float normsq = normsq(a);
    output.x = a.x * invSqrt(normsq) * s;
    output.y = a.y * invSqrt(normsq) * s;
    return output;
}
/* End of 2D Vector Functions */

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

    inline float &operator[](int a)
    {
        return data[a];
    }
};

v3f operator+(v3f a, v3f b)
{
    v3f output;
    output.x = a.x + b.x;
    output.y = a.y + b.y;
    output.z = a.z + b.z;
    return output;
}

v3f operator-(v3f a, v3f b)
{
    v3f output;
    output.x = a.x - b.x;
    output.y = a.y - b.y;
    output.z = a.z - b.z;
    return output;
}

v3f operator*(v3f a, float b)
{
    v3f output;
    output.x = a.x * b;
    output.y = a.y * b;
    output.z = a.z * b;
    return output;
}

v3f operator*(float b, v3f a)
{
    v3f output;
    output.x = a.x * b;
    output.y = a.y * b;
    output.z = a.z * b;
    return output;
}

v3f operator/(v3f a, float b)//Shouldn't need other configuration, since it makes no sense
{
    v3f output;
    output.x = a.x / b;
    output.y = a.y / b;
    output.z = a.z / b;
    return output;
}

float dot(v3f a, v3f b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

v3f cross(v3f a, v3f b)
{
    v3f output;
    output.x = a.y*b.z - b.y*a.z;
    output.y = -a.x*b.z + b.x*a.z;
    output.z = a.x*b.y - b.x*a.y;
    return output;
}

float norm(v3f a)
{
    return sqrt(sq(a.x) + sq(a.y) + sq(a.z));
}

float normSq(v3f a)
{
    return sq(a.x) + sq(a.y)  + sq(a.z);
}

v3f normalize(v3f a)
{
    v3f output;
    output.x = a.x / norm(a);
    output.y = a.y / norm(a);
    output.z = a.z / norm(a);
    return output;
}

v3f project(v3f a, v3f b)
{
    return dot(a, b)/normSq(b) * b;
}

v3f reject(v3f a, v3f b)
{
    return a - project(a, b);
}

float scalarTripleProduct(v3f a, v3f b, v3f c)
{
    return dot(a, cross(b, c));
}

v3f normalizeScale(v3f a, float s)
{
    v3f output;
    float normsq = normsq(a);
    output.x = a.x * invSqrt(normsq) * s;
    output.y = a.y * invSqrt(normsq) * s;
    output.z = a.z * invSqrt(normsq) * s;
    return output;
}
/* End of 3D Vector Functions */
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

    inline float &operator[](int a)
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

    inline float &operator[](int a)
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

    inline float &operator[](int a)
    {
        return data[a];
    }
};

#pragma pack(pop)

float bound(float bounded, float lower, float upper)
{
    if (bounded > upper)
        return upper;
    if (bounded < lower)
        return lower;
    return bounded;
}

float lerp(float a, float b, float t)
{
    if (t > 1.0f) return b;
    if (t < 0.0f) return a;
    return a + (b - a) * t;
}

//x^(-1/2)
float invSqrt(float a)
{
    int isqrti;
    float isqrt = a;

    isqrti = *(int*) &isqrt;
    isqrti = 0x5f375a86 - (isqrti >> 1); //magic

    isqrt = *(float*) &isqrti;
    isqrt *= (1.5f - (a * 0.5f * isqrt * isqrt));
    isqrt *= (1.5f - (a * 0.5f * isqrt * isqrt)); //second itteration, uncomment if greater accuracy is needed
    return isqrt;
}

#endif
