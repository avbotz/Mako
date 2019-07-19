/** @file atmega.hpp
 *  @brief Function definitions for interfacing with the code on the atmega.
 *
 *  @author David Zhang
 */
#ifndef CONTROL_ATMEGA_HPP
#define CONTROL_ATMEGA_HPP 

#include <string>
#include "control/state.hpp"

/** Port where atmega is connected. */
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
	State state();
}

#endif
