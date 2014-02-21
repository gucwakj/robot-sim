#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot, robot1;
	double radius = 4.445;      // radius of 1.75 inches
	double trackwidth = 9.3726;  // the track width, the distance between two wheels
	double x, y;               // x and y coordinates

	/* connect to the paired robot and move to the zero position */
	robot.connect();
	robot1.connect();

	robot.movexyNB(150, 50, radius, trackwidth);
	robot1.movexyNB(250, 50, radius, trackwidth);
	robot.movexyWait();
	robot1.movexyWait();

	/* get the position of the robot */
	robot.getxy(x, y);
	printf("getxy(x, y) = (%lf, %lf)\n", x, y);

	return 0;
}
