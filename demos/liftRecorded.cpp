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
#include <unistd.h>
#include "mobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
CMobot mobot1, mobot2;

CRobotSim sim;
sim.addRobot(mobot1);
sim.addRobot(mobot2);

/* connect to Mobots and move to the zero position at the same time. */
mobot1.connect();
mobot2.connect();
//mobot1.resetToZeroNB();
//mobot2.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();

double timeInterval = 0.1; /* Seconds */
int numDataPoints = 100; /* Unitless */
double time1[numDataPoints];
double time2[numDataPoints];
double angles1[numDataPoints];
double angles2[numDataPoints];
mobot1.recordAngle(MOBOT_JOINT2, time1, angles1, numDataPoints, timeInterval);
mobot2.recordAngle(MOBOT_JOINT3, time2, angles2, numDataPoints, timeInterval);

/* first lift */
//mobot1.moveToNB(0, -90,  0, 0);
//mobot2.moveToNB(0, 0, 90, 0);
//mobot1.moveWait();
//mobot2.moveWait();
//delay(1);

/* second lift */
//mobot1.moveToNB(0, 0, 90,  0);
//mobot2.moveToNB(0,  -90, 0, 0);
//mobot1.moveWait();
//mobot2.moveWait();
//delay(1);

/* move to zero position */
//mobot1.resetToZeroNB(); 
//mobot2.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();

mobot1.recordWait();
mobot2.recordWait();

for (int i = 0; i<numDataPoints; i++) {
    printf("%lf, ", time2[i]);
}
printf("\n");
for (int i = 0; i<numDataPoints; i++) {
    printf("%lf, ", angles1[i]);
}
printf("\n");
for (int i = 0; i<numDataPoints; i++) {
    printf("%lf, ", angles2[i]);
}
printf("\n");

	return 0;
}
