#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	robot.connect();

	//robot.setTwoWheelRobotSpeed(7, 1.75);
	//robot.moveDistance(7, 1.75);
	//robot.moveDistance(-10, 1.75);

	return 0;
}
