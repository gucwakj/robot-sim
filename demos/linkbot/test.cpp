#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotT robot1;

	double x = 0, y = 0, z = 0;

	robot1.connect();
	robot1.resetToZero();
	robot1.moveTo(0, 45, 0);

	robot1.getAccelerometerData(x, y, z);
	printf("x:%9.6lf\ty:%9.6lf\tz:%9.6lf\n", x, y, z);

	//robot1.setExitState(ROBOT_HOLD);
	return 0;
}
