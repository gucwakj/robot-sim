#include <linkbot.h>

CLinkbotI robot;

robot.connect();
robot.resetToZero();
robot.moveTo(45, 0, -45);
robot.setExitState(ROBOT_HOLD);
