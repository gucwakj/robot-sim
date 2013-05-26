#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbot linkbot1;

	CRobotSim sim;
	sim.addRobot(linkbot1);

	linkbot1.connect();

	linkbot1.resetToZero();

	sim.setExitState(1);
	return 0;
}
