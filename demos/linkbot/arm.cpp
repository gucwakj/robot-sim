#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotL robot1, robot2;

	robot1.connect();
	robot2.connect();
	robot1.resetToZero();
	robot2.resetToZero();
	robot1.setColor("green");
	robot2.setColor("red");

	return 0;
}
