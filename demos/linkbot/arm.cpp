#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotL robot1, robot2, robot3;

	robot1.connect("armrc");
	robot2.connect();
	robot3.connect();
	robot1.setColor("green");
	robot2.setColor("red");
	robot3.setColor("blue");
	robot1.resetToZero();
	robot2.resetToZero();
	robot3.resetToZero();

	_simObject->runSimulation();

	return 0;
}
