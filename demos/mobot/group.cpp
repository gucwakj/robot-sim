#include <iostream>
#include "mobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot robot1, robot2;
	CMobotGroup group;

	robot1.connect();
	robot2.connect();

	group.addRobot(robot1);
	group.addRobot(robot2);

	group.resetToZero();

	//group.motionStand();
	group.move(360, 0, 0, 360);
	//delay(3);
	//group.motionUnstand();

	return 0;
}
