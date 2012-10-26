#include <iostream>
#include "mobotfd.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobotFD fd;
	CiMobotSim robot1, robot2;

	fd.addiMobot(robot1);
	fd.addiMobot(robot2, 0, 0.3, 0);
	//fd.addiMobotConnected(robot2, robot1, MOBOT_FACE6, MOBOT_FACE1);

	//robot1.move(0, 45, 45, 45);
	//robot1.moveNB(0, 0, 0, 45);
	//robot1.moveWait();
	//robot1.moveJoint(IMOBOT_JOINT2, 45);
	//robot1.moveJointNB(IMOBOT_JOINT2, 45);
	//robot1.moveJointWait(IMOBOT_JOINT2);
	//robot2.move(0, 45, 65, 45);
	//robot2.moveNB(0, 45, 45, 0);
	//robot2.moveWait();

	return 0;
}
