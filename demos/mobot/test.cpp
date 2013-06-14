#include <iostream>
#include "robotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot mobot1, mobot2/*, mobot3*/;

	CRobotSim sim;
	sim.addRobot(mobot1);
	sim.addRobot(mobot2);
	//sim.addRobot(mobot3);

	mobot1.connect();
	mobot2.connect();

	//mobot1.resetToZero();
	//mobot2.resetToZero();
	//mobot1.moveToNB(0, 45, 0, 0);
	//mobot2.moveTo(0, 0, 0, -15);
	//mobot1.moveWait();
	//mobot2.moveWait();

	sim.setExitState(1);
	return 0;
}
