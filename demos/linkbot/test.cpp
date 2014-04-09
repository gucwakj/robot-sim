#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotL robot1, robot2;

	robot1.connect();
	robot2.connect();
	robot1.resetToZeroNB();
	robot2.resetToZero();
	robot1.moveTo(0, 45, 0);
	robot2.moveToNB(90, 0, 0);
	robot1.moveTo(-45, 45, 0);
	robot1.resetToZeroNB();
	robot2.resetToZero();

	return 0;
}
