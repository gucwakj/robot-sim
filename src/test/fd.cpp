#include <iostream>
#include "robotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CRobotSim sim;
	mobotSim mobot;

	mobot.connect(sim);

	mobot.move(0, 45, -45, 0);

	sim.setExitState(1);
	
	return 0;
}
