#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;

	robot.connect();
	robot.resetToZero();
	robot.move(45, NaN, 45);

	//robot.setExitState(ROBOT_HOLD);

	return 0;
}
