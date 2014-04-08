#include <iostream>
#include "linkbotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
	CLinkbotI robot;
	double radius = 1.75;      // radius of 1.75 inches
	double trackwidth = 3.69;  // the track width, the distance between two wheels
	double speed = 7;        // the speed of the robot in inches per second

	/* connect to the paired robot and move to the zero position */
	robot.connect();

	/* mark points on the floor */
	robot.point(0,  0,   0, 2, "red");
	robot.point(12, 5.5, 0, 2, "green");
	robot.point(24, 30,  0, 2, "blue");
	robot.point(36, 10,  0, 2, "purple");
	robot.point(48, 5,   0, 2, "aqua");

	/* label the points */
	robot.text(12, 5.5, 0.5, "(12,5.5)");
	robot.text(24, 30, 0.5, "(24,30)");
	robot.text(36, 10, 0.5, "(36,10)");
	robot.text(48, 5, 0.5, "(48,5)");

	/* make obstacle course */
	robot.line(-7.2, 0, 0, 9.777, 7.781, 0, 2, "blue");
	robot.line(9.777, 7.781, 0, 23.733, 36.275, 0, 2, "blue");
	robot.line(23.733, 36.275, 0, 37.75, 12.9, 0, 2, "blue");
	robot.line(37.75, 12.90, 0, 48, 8.63, 0, 2, "blue");
	robot.line(7.2, 0, 0, 14.223, 3.219, 0, 2, "blue");
	robot.line(14.223, 3.219, 0, 24.267, 23.728, 0, 2, "blue");
	robot.line(24.267, 23.728, 0, 34.25, 7.10, 0, 2, "blue");
	robot.line(34.25, 7.10, 0, 48, 1.37, 0, 2, "blue");

	/* move to the zero position */
	robot.resetToZero();

	/* set the speed for a two-wheel robot */
	robot.setTwoWheelRobotSpeed(speed, radius);

	/* move the robot to specified points */
	robot.movexy(12, 5.5, radius, trackwidth);
	robot.movexy(24, 30, radius, trackwidth);
	robot.movexy(36, 10, radius, trackwidth);
	robot.movexy(48, 5, radius, trackwidth);

	return 0;
}
