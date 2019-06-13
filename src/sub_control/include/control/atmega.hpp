#ifndef CONTROL_ATMEGA_HPP
#define CONTROL_ATMEGA_HPP 

#include <string>
#include "control/state.hpp"

#define PORT "/dev/ttyACM0"

namespace atmega 
{
	extern FILE *in;
	extern FILE *out;

	void write(std::string);
	void write(const State &);
	void relative(const State &);
	bool alive();
	State state();
}

#endif
