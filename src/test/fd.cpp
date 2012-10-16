#include <iostream>
#include "mobotfd.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobotFD fd;
	CiMobotSim robot1, robot2;

	fd.addiMobot(robot1);
	fd.addiMobot(robot2, 0, 0.3, 0);
	//fd.addiMobotConnected(robot2, robot1, MOBOT_FACE6, MOBOT_FACE1);

	robot1.move(0, 45, 45, 0);
	robot1.moveNB(0, 45, 45, 45);
	robot2.move(0, 56, 76, 9);
	robot1.moveWait();
	robot2.moveWait();

	fd.runSimulation(argc, argv);

	return 0;
}
