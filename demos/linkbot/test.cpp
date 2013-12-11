#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotL robot1, robot2;

	robot1.connect();
	robot2.connect();
	robot1.resetToZeroNB();
	robot2.resetToZero();

	/* inchworm left */
	robot1.moveJointTo(ROBOT_JOINT1, -45);
	//robot2.moveJointTo(ROBOT_JOINT1, 45);
	robot1.moveJointTo(ROBOT_JOINT1, 0);
	//robot2.moveJointTo(ROBOT_JOINT1, 0);

	/* inchworm right */
	//robot2.moveJointTo(ROBOT_JOINT1, 45);
	robot1.moveJointTo(ROBOT_JOINT1, -45);
	//robot2.moveJointTo(ROBOT_JOINT1, 0);
	robot1.moveJointTo(ROBOT_JOINT1, 0);

	return 0;
}
