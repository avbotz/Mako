#ifndef CONTROL_SERVICE_HPP
#define CONTROL_SERVICE_HPP

#include "control/ControlAlive.h"
#include "control/ControlState.h"
#include "control/ControlWrite.h"
#include "control/ControlWriteState.h"

bool alive(control::ControlAlive::Request &, control::ControlAlive::Response &);

bool state(control::ControlState::Request &, control::ControlState::Response &);

bool write(control::ControlWrite::Request &, control::ControlWrite::Response &);

bool writeState(control::ControlWriteState::Request &,
		control::ControlWriteState::Response &);

#endif
