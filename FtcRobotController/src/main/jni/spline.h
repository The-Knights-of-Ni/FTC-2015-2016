#ifndef SPLINE
#define SPLINE
#include maths.h

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
struct quinticSpline //https://www.desmos.com/calculator/ctbggy8gbc
{
	union
	{
		struct
		{
			float Ax;
			float Bx;
			float Cx;
			float Dx;
			float Ex;
			float Fx;
			float Ay;
			float By;
			float Cy;
			float Dy;
			float Ex;
			float Fx;
		};
		float data[12];
	};
	inline float & operator[](int a)
    {
        return data[a];
    }
};
#pragma pack(pop)

cubicSpline cubicSpline(float x0, float y0, float x1, float y1)
{
	cubicSpline solution;
	solution.Dx = x0;
	solution.Dy = y0;
	//TODO: Matrix solve system
	return solution;
}



#endif
