#ifndef COMMON_STATE_HPP 
#define COMMON_STATE_HPP 

#include <string>
const int X = 0;
const int Y = 1;
const int Z = 2;
const int YAW = 3;
const int PITCH = 4;
const int ROLL = 5;
const int N = 6;

struct State 
{
    // N-E-D coordinates.
    float axis[N];
    
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
    
    std::string text() 
    {
        return "(" + std::to_string(axis[0]) + " " + std::to_string(axis[1]) + " " + 
                        std::to_string(axis[2]) + " " + std::to_string(axis[3]) + " " + 
                        std::to_string(axis[4]) + " " + std::to_string(axis[5]) + ")";
    }
};

#endif
