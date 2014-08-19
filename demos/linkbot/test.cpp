#include <iostream>
#include "linkbot.h"
using namespace std;

int main(int argc, char *argv[]) {
	//CLinkbotI robot;
	//robot.connect();
	CLinkbotI robot, robot1;
	CLinkbotIGroup group;
	group.addRobot(robot);
	group.addRobot(robot1);

	group.connect();
	group.driveDistance(3, 1.75);
	return 0;
}
