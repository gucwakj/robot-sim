#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	double radius=1.75; // the radius of the two wheels of the robot in inches
	double speed=7;   // the speed in 2.5 inches per second for a two-wheel robot
	double distance=5;  // the distance in 3 inches to move forward

	robot.connect();
	robot.resetToZero();
	robot.setTwoWheelRobotSpeed(speed, radius);
	//robot.moveForward(360);
	//robot.moveBackward(360);
	robot.moveDistance(12, radius);
	robot.moveDistance(-12, radius);
	//robot.moveDistance(distance, radius);

	return 0;
}
