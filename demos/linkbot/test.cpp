#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	robot.connect();
	robot.setTwoWheelRobotSpeed(5, 1.75);

	robot.moveTo(179, 0, -179);

	return 0;
}
