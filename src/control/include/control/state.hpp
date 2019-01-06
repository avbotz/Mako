#ifndef COMMON_STATE_HPP 
#define COMMON_STATE_HPP 

#include <string>
#define X 0
#define Y 1
#define Z 2
#define YAW 3
#define PITCH 4
#define ROLL 5
#define N 6

struct State 
{
    // N-E-D coordinates.
	State() {}
    State(float x, float y, float z, float yaw, float pitch, float roll) 
    {
        axis[X] = x;
        axis[Y] = y;
        axis[Z] = z;
        axis[YAW] = yaw;
        axis[PITCH] = pitch;
        axis[ROLL] = roll;
    }

    float axis[N];
};

#endif
