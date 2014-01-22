#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();
	//robot1.setTwoWheelRobotSpeed(7, 1.75);
	robot1.setJointSpeed(ROBOT_JOINT1, 240);
	robot1.setJointSpeed(ROBOT_JOINT3, 240);
	robot1.moveForward(360);
	robot1.moveBackward(360);
	//robot1.moveDistance(5, 1.75);

	return 0;
}
