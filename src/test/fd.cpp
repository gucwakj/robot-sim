#include <iostream>
#include "mobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot mobot1/*, mobot2*/;

	CRobotSim sim;
	sim.addRobot(mobot1);
	//sim.addRobot(mobot2);

	mobot1.connect();
	//mobot2.connect();
	//mobot.move(0, 45, -45, 0);

	sim.setExitState(1);
	return 0;
}
