#include "imobotsim.h"

dReal ang[] = {	0,	0,	90,	0,
				0,	85, 123456789, 0,
				0,	85, 0, 0};
dReal vel[] = {1};
vel[10] = 0.1;

int main(int argc, char* argv[]) {
	CiMobotSim *sim = new CiMobotSim(1, 3, 1, ang);
	sim->setAngVel(vel);

	sim->groundPlane(0, 0, 0, 1, 0);
	sim->iMobotBuild(0, 0, 0, 0);

	sim->runSimulation(argc, argv);
	printf("Reply Message: %d\n", sim->getReplyMessage());

	delete sim;
	return 0;
}
