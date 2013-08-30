#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1, robot2;

	robot1.connect();
	robot2.connect();
	robot1.resetToZero();
	robot2.resetToZero();

	//delay(1);
	robot1.setColorRGB(255, 0, 0);
	robot1.blinkLED(500, 3);

	robot1.setExitState(ROBOT_HOLD);

	return 0;
}
