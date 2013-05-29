#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotL linkbot;

	CRobotSim sim;
	sim.addRobot(linkbot);

	linkbot.connect();
	//linkbot.resetToZero();
	linkbot.move(45, 45, 45);

	sim.setExitState(1);
	return 0;
}
