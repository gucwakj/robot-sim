#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot1;

	robot1.connect();
	robot1.resetToZero();

	robot1.setExitState(ROBOT_HOLD);

	return 0;
}
