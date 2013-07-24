#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotT robot1, robot2;
	CLinkbotTGroup group;

	robot1.connect();
	robot2.connect();

	group.addRobot(robot1);
	group.addRobot(robot2);

	group.resetToZero();

	group.move(36, 0, 36);

	return 0;
}
