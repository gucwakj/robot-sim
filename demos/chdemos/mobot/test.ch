#include <mobot.h>

CMobot robot;

robot.connect();

robot.resetToZero();
robot.setJointSpeeds(120, 120, 120, 120);
//robot.moveDistance(14, 1.75);
robot.moveTo(345, 0, 0, 45);

//robot.setExitState(ROBOT_HOLD);
