#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();
	//robot1.resetToZero();
	robot1.moveTo(36, 0, -36);

	//double x, y, z;
	//robot1.getAccelerometerData(x, y, z);
	//printf("x: %lf y: %lf z: %lf\n", x, y, z);

	//robot1.setExitState(ROBOT_HOLD);
	return 0;
}
