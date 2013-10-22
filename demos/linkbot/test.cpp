#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotL robot1, robot2, robot3, robot4;

	robot1.connect();
	robot2.connect();
	robot3.connect();
	robot4.connect();
	//robot1.resetToZero();
	robot1.moveTo(36, 0, -36);

	//double x, y, z;
	//robot1.getAccelerometerData(x, y, z);
	//printf("x: %lf y: %lf z: %lf\n", x, y, z);

	robot1.setExitState(ROBOT_HOLD);
	return 0;
}
