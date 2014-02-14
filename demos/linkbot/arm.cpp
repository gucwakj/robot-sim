#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotL robot1, robot2, robot3;

	robot1.connect("armrc");
	robot1.setColor("green");
	robot2.connect();
	robot2.setColor("red");
	robot3.connect();
	robot3.setColor("blue");

	robot1.point(0, 10, 8, 1, "blue");

	robot1.resetToZero();
	robot2.resetToZero();
	robot3.resetToZero();

	robot3.setGoal(0, 0.5, 0.5);

	_simObject->runSimulation();

	return 0;
}
