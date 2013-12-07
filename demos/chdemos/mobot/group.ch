#include "mobot.h"

CMobot robot1, robot2;
CMobotGroup group;

robot1.connect();
robot2.connect();

group.addRobot(robot1);
group.addRobot(robot2);

group.resetToZero();

group.move(360, 0, 0, 360);
