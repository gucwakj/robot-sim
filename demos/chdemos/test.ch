#include "robotsim.h"

CMobot mobot1;

CRobotSim sim;
sim.addRobot(mobot1);

mobot1.connect();
mobot1.resetToZero();
mobot1.moveTo(0, 0, 45, 0);

sim.setExitState(1);
