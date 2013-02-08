/* Filename: dataAcquisition.cpp
 * Make a graph of the mobot's joint angle versus time */

#include <iostream>
#include "robotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CRobotSim sim;
	CMobot mobot;
	
	mobot.connect(sim);

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

	/* Start the motion. First, move mobot to zero position */
	mobot.resetToZero();

	/* Set the joint 1 speed to 45 degrees/second */
	mobot.setJointSpeed(MOBOT_JOINT1, speed);
	mobot.setJointSpeed(MOBOT_JOINT4, speed);

	/* Start capturing data */
	mobot.recordAngle(MOBOT_JOINT1, time, angles1, numDataPoints, timeInterval);

	/* Move the joint 720 degrees */
	mobot.move(angle, 0, 0, angle);

	/* Wait for recording to finish */
	mobot.recordWait();

	printf("data: num points = %d\ttime = %lf\n", numDataPoints, movementTime);
	for (int i = 0; i<numDataPoints; i++) {
	    printf("%lf, ",time[i]);
	}
	printf("\n");
	for (int i = 0; i<numDataPoints; i++) {
	    printf("%lf, ",angles1[i]);
	}
	printf("\n");

	return 0;
}
