#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;

	robot.connect();
	robot.moveNB(180, 0, -180);
	while (robot.isMoving()) {
		printf("moving\n");
	}
	printf("done moving\n");

	return 0;
}
