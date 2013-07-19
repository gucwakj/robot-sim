/* File: liftRecorded.cpp
   Lift two connected Mobots.
   Joint 4 of the 1st Mobot should be connected to 
   Joint 1 of the 2nd Mobot as
           1st                       2nd
  |---------|--------|      |---------|--------|     
 1|   2     |   3    | 4 X 1|    2    |   3    | 4
  |---------|--------|      |---------|--------|
*/

#include <iostream>
#include "mobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobot robot1, robot2;

	/* connect to Mobots and move to the zero position at the same time. */
	robot1.connect();
	robot2.connect();
	robot1.resetToZeroNB();
	robot2.resetToZeroNB();
	robot1.moveWait();
	robot2.moveWait();

	double timeInterval = 0.1; /* Seconds */
	int numDataPoints = 100; /* Unitless */
	double time1[numDataPoints];
	double time2[numDataPoints];
	double angles1[numDataPoints];
	double angles2[numDataPoints];
	robot1.recordAngle(ROBOT_JOINT2, time1, angles1, numDataPoints, timeInterval);
	robot2.recordAngle(ROBOT_JOINT3, time2, angles2, numDataPoints, timeInterval);

	/* first lift */
	robot1.moveToNB(0, -90,  0, 0);
	robot2.moveToNB(0, 0, 90, 0);
	robot1.moveWait();
	robot2.moveWait();
	//delay(1);

	/* second lift */
	robot1.moveToNB(0, 0, 90,  0);
	robot2.moveToNB(0,  -90, 0, 0);
	robot1.moveWait();
	robot2.moveWait();
	//delay(1);

	/* move to zero position */
	robot1.resetToZeroNB(); 
	robot2.resetToZeroNB();
	robot1.moveWait();
	robot2.moveWait();

	robot1.recordWait();
	robot2.recordWait();

	for (int i = 0; i < numDataPoints; i++) {
		printf("%lf, ", time2[i]);
	}
	printf("\n");
	for (int i = 0; i < numDataPoints; i++) {
		printf("%lf, ", angles1[i]);
	}
	printf("\n");
	for (int i = 0; i < numDataPoints; i++) {
		printf("%lf, ", angles2[i]);
	}
	printf("\n");

	return 0;
}
