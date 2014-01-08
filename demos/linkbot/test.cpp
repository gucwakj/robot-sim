#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();

	robot1.moveForward(180);
	robot1.turnLeft(90, 1.75, 3.69);
	robot1.moveForward(180);
	robot1.turnRight(90, 1.75, 3.69);
	robot1.moveForward(180);

	return 0;
}
