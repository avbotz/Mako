/** @file state.hpp
 *  @brief State struct and constant definitions.
 *
 *  @author David Zhang
 */
#ifndef COMMON_STATE_HPP 
#define COMMON_STATE_HPP 

#include <string>
#include <sstream>

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
		std::ostringstream os;
		os.precision(2);
		os << std::fixed;
		os << "(";
		for (int i = 0; i < N-1; i++) os << axis[i] << " "; os << axis[N-1];
		os << ")";
		return os.str();
	}
};

#endif
