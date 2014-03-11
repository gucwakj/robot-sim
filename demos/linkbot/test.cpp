#include <iostream>
#include "linkbotsim.h"
using namespace std;

double func(double x) {
	return x*x;
}

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	double radius = 4.445;      // radius of 1.75 inches
	double trackwidth = 9.3726;  // the track width, the distance between two wheels

	robot.connect();
	robot.movexyFunc(0, 4, 10, func, radius, trackwidth);
	robot.movexyWait();
	return 0;
}
