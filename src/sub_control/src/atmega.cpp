#include <iostream>
#include <cmath>
#include <thread>
#include "control/state.hpp"
#include "control/atmega.hpp"
#include "vision/config.hpp"


namespace atmega 
{    
	FILE* in = fopen(PORT, "r+");
	FILE* out = fopen(PORT, "w+");
	State sim_state(0, 0, 0, 0, 0, 0);

	void write(std::string command)
	{
		// Don't forget to add a \n to a command sent to this function.
		if (!SIM) 
		{
			fprintf(out, command.c_str());
			fflush(out);
		}
	}

	void write(const State &state) 
	{
		// Write to pipe(out) with state data and flush.
		if (!SIM) 
		{
			fprintf(out, "s %f %f %f %f %f %f\n", state.axis[X], state.axis[Y], 
					state.axis[Z], state.axis[YAW], state.axis[PITCH], state.axis[ROLL]);
			fflush(out);
		}
		else 
		{
			sim_state = state;	
		}
	}

	void relative(const State &state)
	{
		// Write to pipe(out) with relative data and flush.
		if (!SIM)
		{
			fprintf(out, "s %f %f %f %f %f %f\n", state.axis[X], state.axis[Y], 
					state.axis[Z], state.axis[YAW], state.axis[PITCH], state.axis[ROLL]);
			fflush(out);
		}
	}

	bool alive()
	{
		if (SIM) return true;

		// Request for kill. 
		write("a\n");

		// Read from in pipe.
		int kill;
		fscanf(in, "%i", &kill);

		return kill != 0;			
	}

	State state()
	{	
		if (SIM) return sim_state;

		// Request for state.
		write("c\n");

		// Read from in pipe, using %f for floats.
		float x, y, z, yaw, pitch, roll;
		fscanf(in, "%f %f %f %f %f %f", &x, &y, &z, &yaw, &pitch, &roll);

		State state(x, y, z, yaw, pitch, roll);
		return state;
	}
}
