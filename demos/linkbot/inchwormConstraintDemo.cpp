#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotL robot1, robot2;

	robot1.connect();
	robot2.connect();
	robot1.resetToZero();
	robot2.resetToZero();
	robot1.setColor("green");
	robot2.setColor("red");

	printf("setting\n");
	robot2.setJointSpeed(ROBOT_JOINT1, 45);
	printf("setting\n");
	robot2.moveJoint(ROBOT_JOINT1, 90);
	printf("setting\n");

	while (1) { usleep(1000); }

	return 0;
}
