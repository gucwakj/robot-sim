#include "robotsim.h"

CLinkbotI linkbot1;

CRobotSim sim;
sim.addRobot(linkbot1);

linkbot1.connect();
linkbot1.resetToZero();
linkbot1.moveTo(0, 0, 45);

sim.setExitState(1);
