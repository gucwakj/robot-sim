#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;

	robot.connect();
	robot.traceOn();
	robot.moveDistance(6, 2);
	robot.traceOff();
	robot.moveDistance(6, 2);

	return 0;
}
