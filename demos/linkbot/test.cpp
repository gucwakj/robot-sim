#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1, robot2, robot3, robot4;
	CLinkbotL gripper;

	robot1.connect();
	robot2.connect();
	robot3.connect();
	robot4.connect();
	gripper.connect();

	return 0;
}
