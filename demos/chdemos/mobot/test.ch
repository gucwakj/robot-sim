#include <mobotsim.h>

CMobot robot;

robot.connect();

robot.resetToZero();
robot.setJointSpeeds(120, 120, 120, 120);
robot.moveDistance(14, 1.75);

robot.setExitState(ROBOT_HOLD);
