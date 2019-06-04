#include <iostream>
#include <cmath>
#include <thread>
#include "control/state.hpp"
#include "control/atmega.hpp"


namespace atmega 
{    
	FILE* in = fopen(PORT, "r+");
	FILE* out = fopen(PORT, "w+");

	void write(std::string command)
	{
		// Don't forget to add a \n to a command sent to this function.
		fprintf(out, command.c_str());
		fflush(out);
	}

	void write(const State &state) 
	{
		// Write to pipe(out) with state data and flush.
		fprintf(out, "s %f %f %f %f %f %f\n", state.axis[X], state.axis[Y], 
                state.axis[Z], state.axis[YAW], state.axis[PITCH], state.axis[ROLL]);
		fflush(out);
	}

	bool alive()
	{
        // Request for kill. 
        write("a\n");
        
        // Read from in pipe.
        int kill;
        fscanf(in, "%i", &kill);

        return kill != 0;			
	}
	
	State state()
	{	
        // Request for state.
        write("c\n");
        
        // Read from in pipe, using %f for floats.
        float x, y, z, yaw, pitch, roll;
        fscanf(in, "%f %f %f %f %f %f", &x, &y, &z, &yaw, &pitch, &roll);

        State state(x, y, z, yaw, pitch, roll);
        return state;
	}
}
