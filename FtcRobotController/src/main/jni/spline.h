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
struct quinticSpline //https://www.desmos.com/calculator/ctbggy8gbc
{
	union
	{
		struct
		{
			float a;
			float b;
			float c;
			float d;
			float e;
		};
		float data[6];
	};
	inline float & operator[](int a)
    {
        return data[a];
    }
	float x_offset;
	float y_offset;
	float knot_distance;
	float theta_offset;
	float arc_length;
	float yp0_hat;
	float yp1_hat;
	float theta0_hat;
	float theta1_hat;
	float x1_hat;

	quinticSpline(float x0, float y0, float theta_0, float x1, float y1, float theta_1)//TODO: Add safetys so we don't crash
	{

		x_offset = x0;
	    y_offset = y0;

	    x1_hat = sqrt((x1 - x0) * (x1 - x0) + (y1 - y0) * (y1 - y0));
		if (x1_hat == 0) {
      		printf("X1 Hat is 0, cannot handle this\n");
    	}

	    knot_distance = x1_hat;
	    theta_offset = atan2(y1 - y0, x1 - x0);
	    theta0_hat = getDifferenceInAngleRadians(theta_offset, theta_0);
	    theta1_hat = getDifferenceInAngleRadians(theta_offset, theta_1);
		yp0_hat = tan(theta0_hat);
	    yp1_hat = tan(theta1_hat);

		a = -(3 * (yp0_hat + yp1_hat)) / (x1_hat * x1_hat * x1_hat * x1_hat);
	    b = (8 * yp0_hat + 7 * yp1_hat) / (x1_hat * x1_hat * x1_hat);
	    c = -(6 * yp0_hat + 4 * yp1_hat) / (x1_hat * x1_hat);
	    d = 0;
	    e = yp0_hat;
	}
};
#pragma pack(pop)




#endif
