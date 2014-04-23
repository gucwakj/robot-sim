#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;

	robot.connect();
	robot.movexy(1, 1, 1.75, 3);
	robot.movexy(0, 1, 1.75, 3);

	return 0;
}
