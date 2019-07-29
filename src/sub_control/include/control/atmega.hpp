/** @file atmega.hpp
 *  @brief Function definitions for interfacing with the code on the atmega.
 *
 *  Nothing from mission or vision should depend on this file. Instead, use the
 *  control service to ensure that there aren't multiple clients writing to
 *  Nautical at the same time.
 *
 *  @author David Zhang
 */
#ifndef CONTROL_ATMEGA_HPP
#define CONTROL_ATMEGA_HPP 

#include "control/state.hpp"

#define PORT "/dev/ttyACM0"

namespace atmega 
{
	extern FILE *in;
	extern FILE *out;
	extern State sim_state;

	void write(std::string);
	void write(const State &);
	void relative(const State &);
	bool alive();
	float depth();
	State state();
}

#endif
