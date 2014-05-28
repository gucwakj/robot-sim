#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;

	robot.connect();
	robot.resetToZero();
	robot.setTwoWheelRobotSpeed(7, 1.75);
	robot.movexyTo(3, 12, 1.75, 0);

	return 0;
}
