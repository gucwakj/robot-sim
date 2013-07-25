#include <iostream>
#include "mobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot robot, robot2;

	robot.connect();
	robot2.connect();
	robot.resetToZero();
	robot2.resetToZero();
	robot.moveDistance(6, 1.75);

	return 0;
}
