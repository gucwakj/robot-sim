#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	//CLinkbotI robot, robot2;

	robot.connect();
	//robot2.connect();
	robot.resetToZero();
	//robot.move(45, 0, -45);

	robot.setExitState(ROBOT_HOLD);

	return 0;
}
