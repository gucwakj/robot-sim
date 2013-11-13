#include <iostream>
#include "linkbotsim.h"
using namespace std;

void function(void) {
	CLinkbotI robot;
	robot.connect();
	robot.moveTo(36, 0, -36);
}

int main(int argc, char *argv[]) {
	CLinkbotI robot;

	robot.connect();
	robot.moveTo(36, 0, -36);

	function();

	robot.moveTo(72, 0, -72);

	return 0;
}
