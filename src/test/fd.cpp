#include <iostream>
#include "mobotfd.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobotFD fd;
	//Mobot robot1(0), robot2(0);
	//CMobotSim robot1, robot2;
	CiMobotSim robot1, robot2;

	//double ang[] = { 0, 45, 45, -45, 45, 45, 45, 0};
	//fd.setAngles(ang);

	fd.addiMobot(robot1);
	fd.addiMobot(robot2);
	//fd.addiMobotConnected(robot2, robot1, MOBOT_FACE6, MOBOT_FACE1);

	robot1.move(0, 45, 45, 0);
	robot1.moveNB(0, 45, 45, 45);
	robot2.move(0, 56, 76, 9);
	robot1.moveWait();
	robot2.moveWait();

	//fd.runSimulation(argc, argv);
	//cout << "Reply Message: " << fd.getReplyMessage() << endl;

	return 0;
}
