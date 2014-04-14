#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
CLinkbotI robot;

robot.connect();

double speed1 = 375;
double speed2;
double speed3 = 375;
robot.setJointSpeeds(speed1, NaN, speed3);

/* move the joint 1 by 180 degrees and joint 3 by -180 degrees */
robot.move(180, 0, -180);

/* get the joint speed speed for all joints */
robot.getJointSpeeds(speed1, speed2, speed3);
printf("Joint1 speed = %.2lf degrees per second\n", speed1);
printf("Joint3 speed = %.2lf degrees per second\n", speed3);

	return 0;
}
