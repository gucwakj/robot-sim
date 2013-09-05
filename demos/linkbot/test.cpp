#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotT robot1, robot2;

	robot1.connect();
	robot2.connect();
	robot1.resetToZero();
	robot2.resetToZero();
	//robot1.moveTo(45, 45, 45);

	double x, y, z;
	robot1.getAccelerometerData(x, y, z);
	printf("x: %lf y: %lf z: %lf\n", x, y, z);
	//robot1.setExitState(ROBOT_HOLD);
	return 0;
}
