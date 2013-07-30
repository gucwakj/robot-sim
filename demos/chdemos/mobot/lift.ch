/* File: lift.ch
   Lift two connected Mobots.
   Joint 4 of the 1st Mobot should be connected to 
   Joint 1 of the 2nd Mobot as
           1st                       2nd
  |---------|--------|      |---------|--------|     
 1|   2     |   3    | 4 X 1|    2    |   3    | 4
  |---------|--------|      |---------|--------|
*/
#include <mobot.h>
#include <chplot.h>
#include <numeric.h>

/* Declare plotting variables */
CPlot plot1, plot2, plot3;

/* Declare mobots */
CMobot mobot1, mobot2;

/* connect to Mobots and move to the zero position at the same time. */
mobot1.connect();
mobot2.connect();
mobot1.resetToZeroNB();
mobot2.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();

double timeInterval = 0.1; /* Seconds */
int numDataPoints = 100; /* Unitless */
double time1[numDataPoints];
double time2[numDataPoints];
double angles1[numDataPoints];
double angles2[numDataPoints];
mobot1.recordAngle(ROBOT_JOINT2, time1, angles1, numDataPoints, timeInterval);
mobot2.recordAngle(ROBOT_JOINT2, time2, angles2, numDataPoints, timeInterval);
/* first lift */
mobot1.moveToNB(0, -90,  0, 0);
mobot2.moveToNB(0, 0, 90, 0);
mobot1.moveWait();
mobot2.moveWait();
//delay(1);

/* second lift */
mobot1.moveToNB(0, 0, 90,  0);
mobot2.moveToNB(0,  -90, 0, 0);
mobot1.moveWait();
mobot2.moveWait();
//delay(1);

/* move to zero position */
mobot1.resetToZeroNB(); 
mobot2.resetToZeroNB();
mobot1.moveWait();
mobot2.moveWait();

mobot1.recordWait();
mobot2.recordWait();

plot1.title("Data for Joint Angle 2 versus Time");
plot1.label(PLOT_AXIS_X, "Time (seconds)");
plot1.label(PLOT_AXIS_Y, "Angle (degrees)");
plot1.data2DCurve(time1, angles1, numDataPoints);
plot1.data2DCurve(time2, angles2, numDataPoints);
plot1.lineType(0, 1, 2, "blue");
plot1.lineType(1, 1, 2, "green");
plot1.legend("Mobot 1", 0);
plot1.legend("Mobot 2", 1);
plot1.grid(PLOT_ON);
plot1.plotting();
