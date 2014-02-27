#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	double radius = 4.445;      // radius of 1.75 inches
	double trackwidth = 9.3726;  // the track width, the distance between two wheels

	robot.connect();
	robot.moveFunction("y=5x+3", 0, 4, radius);
	return 0;
}
