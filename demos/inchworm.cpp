#include "imobotsim.h"

dReal ang[] = {	0,	-60,	0,	0,
				0,	0,		-60, 0,
				0, 0, 0, 0};

int main(int argc, char* argv[]) {
	CiMobotSim *sim = new CiMobotSim(1, 3, 1, ang);

	sim->groundPlane(0, 0, 0, 1, 0);
	sim->iMobotBuild(0, 0, 0, 0);

	sim->runSimulation(argc, argv);
	printf("Reply Message: %d\n",sim->getReplyMessage());

	delete sim;
	return 0;
}
