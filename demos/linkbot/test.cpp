#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	//CLinkbotI robot1;
	CLinkbotI robot1, robot2, robot3, robot4, robot5;

	robot1.connect();
	robot2.connect();
	//robot3.connect();
	//robot4.connect();
	//robot5.connect();
	robot1.resetToZero();
	robot2.resetToZero();
	//robot3.resetToZero();
	//robot4.resetToZero();
	//robot5.resetToZero();

	robot1.moveToNB(-10, NaN, -30);
	robot2.moveToNB(95, NaN, -95);
	//robot2.moveToNB(-95, NaN, 95);

	robot1.setExitState(ROBOT_HOLD);

	return 0;
}
