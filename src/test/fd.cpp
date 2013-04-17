#include <iostream>
#include "mobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot mobot1/*, mobot2, mobot3*/;

	CRobotSim sim;
	sim.addRobot(mobot1);
	//sim.addRobot(mobot2);
	//sim.addRobot(mobot3);

	//mobot1.connect();
	//mobot2.connect();

	//mobot1.motionStand();
	mobot1.resetToZero();
	mobot1.moveTo(0, 45, 0, 0);
	mobot1.resetToZeroNB();
	//mobot1.moveWait();
	//mobot2.moveTo(0, 0, 0, -15);
	//mobot1.moveWait();
	//mobot2.moveWait();

	//sim.setExitState(1);
	return 0;
}
