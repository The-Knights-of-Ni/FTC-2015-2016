#ifndef SPLINE
#define SPLINE
#include "maths.h"

#pragma pack(push, 16)
struct cubicSpline //Cleaner than using a couple vectors or whatever
{
	union
	{
		struct
		{
			float Ax;
			float Bx;
			float Cx;
			float Dx;
			float Ay;
			float By;
			float Cy;
			float Dy;
		};
		float data[8];
	};
	inline float & operator[](int a)
    {
        return data[a];
    }
};
#pragma pack(pop)




#endif
