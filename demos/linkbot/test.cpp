#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbot linkbot1;

	CRobotSim sim;
printf("sim created\n");
	sim.addRobot(linkbot1);
printf("robot added\n");
	//sim.addRobot(mobot2);
	//sim.addRobot(mobot3);

	//mobot1.connect();
	//mobot2.connect();

	linkbot1.resetToZero();
	//mobot1.moveTo(0, 45, 0, 0);
	//mobot2.moveTo(0, 0, 0, -15);
	//mobot1.moveWait();
	//mobot2.moveWait();

	sim.setExitState(1);
	return 0;
}
