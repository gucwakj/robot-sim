/* Filename: dataAcquisition.cpp
 * Make a graph of the robot's joint angle versus time */

#include <iostream>
#include "mobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot robot;

	robot.connect();

	double speed = 45; /* Degrees/second */
	double angle = 720; /* Degrees */
	double timeInterval = 0.1; /* Seconds */

	/* Figure out how many data points we will need. First, figure out the
	 * approximate amount of time the movement should take. */
	double movementTime = angle / speed; /* Seconds */
	/* Add an extra second of recording time to make sure the entire movement is
	 * recorded */
	movementTime = movementTime + 1; 
	int numDataPoints = movementTime / timeInterval; /* Unitless */

	/* Initialize the arrays to be used to store data for time and angle */
	double time[numDataPoints];
	double angles1[numDataPoints];

	/* Start the motion. First, move robot to zero position */
	robot.resetToZero();

	/* Set the joint 1 speed to 45 degrees/second */
	robot.setJointSpeed(ROBOT_JOINT1, speed);
	robot.setJointSpeed(ROBOT_JOINT4, speed);

	/* Start capturing data */
	robot.recordAngle(ROBOT_JOINT1, time, angles1, numDataPoints, timeInterval);

	/* Move the joint 720 degrees */
	robot.move(angle, 0, 0, angle);

	/* Wait for recording to finish */
	robot.recordWait();

	return 0;
}
