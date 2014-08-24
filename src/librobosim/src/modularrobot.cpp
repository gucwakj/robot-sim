#include "modularrobot.hpp"
#include "robosim.hpp"

ModularRobot::ModularRobot(void) : Robot(JOINT1, JOINT1) { }

ModularRobot::~ModularRobot(void) {
	// destroy connectors array
	for (int i = 0; i < _conn.size(); i++) {
		delete _conn[i];
	}
}

int ModularRobot::connect(char *name, int pause) {
	// create simulation object if necessary
	if (!g_sim)
		g_sim = new RoboSim(name, pause);

	// set initial 'led' color
	_rgb[0] = 0;
	_rgb[1] = 1;
	_rgb[2] = 0;

	// add to simulation
	g_sim->addRobot(this);

	// and we are connected
	_connected = 1;

	// success
	return 0;
}

/**********************************************************
	protected functions for inherited classes
 **********************************************************/
dBodyID ModularRobot::getConnectorBodyID(int face) {
	for (int i = 0; i < _conn.size(); i++) {
		if (_conn[i]->face == face) return _conn[i]->body;
	}
	return NULL;
}

