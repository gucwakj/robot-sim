#include "linkbotsim.h"

CLinkbotT robot1, robot2;
CLinkbotTGroup group;

robot1.connect();
robot2.connect();

group.addRobot(robot1);
group.addRobot(robot2);

group.resetToZero();

//group.motionStand();
group.move(36, 0, 36);
//delay(3);
//group.motionUnstand();
