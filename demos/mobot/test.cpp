#include <iostream>
#include "mobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot robot;

	robot.connect();
	robot.resetToZero();
	//robot.moveDistance(6, 1.75);

	robot.setExitState(ROBOT_HOLD);
	return 0;
}
