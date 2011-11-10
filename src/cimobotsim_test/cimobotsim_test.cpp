#include "imobotsim.h"

dReal ang[] = {	0,	45,	45,	0};

int main(int argc, char* argv[]) {
	CiMobotSim *sim = new CiMobotSim(1, 1, 1, ang);

	//sim->setCOR(0.45, 0.45);
	//sim->setMu(0.3, 0.1);
	//sim->setTime(1.0);
	//sim->setAngVel(vel);

	sim->groundPlane(0, 0, 0, 1, 0);

    sim->iMobotAnchor(0, LE, 0, 0, 0, 0, 90, 0, 0, 0, 0, 0);
	//sim->iMobotBuild(0, 0, 0, 0);
	//sim->iMobotBuild(0, 0, 0, 5, 15, 25, 45);
	//sim->iMobotBuild(0, 0, 0, 5, 0, 0, 0, 0, 45, 45, 0);

	sim->runSimulation(argc, argv);
	printf("Reply Message: %d\n",sim->getReplyMessage());

	delete sim;
	return 0;
}
