#ifndef CONTROL_ATMEGA_HPP
#define CONTROL_ATMEGA_HPP 

#include <string>
#include "control/state.hpp"

#define PORT "mock_port"

namespace atmega 
{
	extern FILE *in;
	extern FILE *out;

	void write(std::string);
	void write(const State&);
	bool read_alive();
	State read_state();
}

#endif
