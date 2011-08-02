#include "imobotsim.h"

dReal ang[] = {	0,	0,	0,	0,			0,	0,	0,	0};

int main(int argc, char* argv[]) {
	//CiMobotSim *sim = new CiMobotSim(1, 2, 1, 1.0, ang);
	CiMobotSim *sim = new CiMobotSim(2, 1, 1, 1.0, ang);

	sim->setMu(0.3, 0.1);
	sim->setCOR(0.45, 0.45);

	sim->groundPlane(0, 0, 0, 1, 0);

	//sim->iMobotBuild(0, 0, 0, 5);
	//sim->iMobotBuild(0, 0, 0, 5, 15, 25, 45);
	//sim->iMobotBuild(0, 0, 0, 5, 15, 25, 45, 0, 0, 0, 0);

	//sim->iMobotBuild(0, 0, 0, 4);
	//sim->iMobotBuild(0, 0, 0, 4, 0, 0, 0);
	sim->iMobotBuild(0, 0, 0, 4, 0, 0, 0, 0, 45, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 1, 1);
	//sim->iMobotBuildAttached(1, 0, 1, 2);
	//sim->iMobotBuildAttached(1, 0, 1, 3);
	//sim->iMobotBuildAttached(1, 0, 1, 4);
	//sim->iMobotBuildAttached(1, 0, 1, 5);
	//sim->iMobotBuildAttached(1, 0, 1, 6);
	sim->iMobotBuildAttached(1, 0, 2, 1);
	//sim->iMobotBuildAttached(1, 0, 2, 2);
	//sim->iMobotBuildAttached(1, 0, 2, 3);
	//sim->iMobotBuildAttached(1, 0, 2, 4);
	//sim->iMobotBuildAttached(1, 0, 2, 5);
	//sim->iMobotBuildAttached(1, 0, 2, 6);
	//sim->iMobotBuildAttached(1, 0, 3, 1);
	//sim->iMobotBuildAttached(1, 0, 3, 2);
	//sim->iMobotBuildAttached(1, 0, 3, 3);
	//sim->iMobotBuildAttached(1, 0, 3, 4);
	//sim->iMobotBuildAttached(1, 0, 3, 5);
	//sim->iMobotBuildAttached(1, 0, 3, 6);
	//sim->iMobotBuildAttached(1, 0, 4, 1);
	//sim->iMobotBuildAttached(1, 0, 4, 2);
	//sim->iMobotBuildAttached(1, 0, 4, 3);
	//sim->iMobotBuildAttached(1, 0, 4, 4);
	//sim->iMobotBuildAttached(1, 0, 4, 5);
	//sim->iMobotBuildAttached(1, 0, 4, 6);
	//sim->iMobotBuildAttached(1, 0, 5, 1);
	//sim->iMobotBuildAttached(1, 0, 5, 2);
	//sim->iMobotBuildAttached(1, 0, 5, 3);
	//sim->iMobotBuildAttached(1, 0, 5, 4);
	//sim->iMobotBuildAttached(1, 0, 5, 5);
	//sim->iMobotBuildAttached(1, 0, 5, 6);
	//sim->iMobotBuildAttached(1, 0, 6, 1);
	//sim->iMobotBuildAttached(1, 0, 6, 2);
	//sim->iMobotBuildAttached(1, 0, 6, 3);
	//sim->iMobotBuildAttached(1, 0, 6, 4);
	//sim->iMobotBuildAttached(1, 0, 6, 5);
	//sim->iMobotBuildAttached(1, 0, 6, 6);
	//sim->iMobotBuildAttached(1, 0, 1, 1, 45, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 1, 2, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 1, 3, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 1, 4, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 1, 5, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 1, 6, 0, 0, 45, 45);
	//sim->iMobotBuildAttached(1, 0, 2, 1, 45, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 2, 2, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 2, 3, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 2, 4, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 2, 5, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 2, 6, 0, 0, 45, 45);
	//sim->iMobotBuildAttached(1, 0, 3, 1, 45, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 3, 2, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 3, 3, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 3, 4, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 3, 5, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 3, 6, 0, 0, 45, 45);
	//sim->iMobotBuildAttached(1, 0, 4, 1, 45, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 4, 2, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 4, 3, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 4, 4, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 4, 5, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 4, 6, 0, 0, 45, 45);
	//sim->iMobotBuildAttached(1, 0, 5, 1, 45, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 5, 2, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 5, 3, 0, 45, 0, 0);
 	//sim->iMobotBuildAttached(1, 0, 5, 4, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 5, 5, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 5, 6, 0, 0, 45, 45);
	//sim->iMobotBuildAttached(1, 0, 6, 1, 45, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 6, 2, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 6, 3, 0, 45, 0, 0);
	//sim->iMobotBuildAttached(1, 0, 6, 4, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 6, 5, 0, 0, 45, 0);
	//sim->iMobotBuildAttached(1, 0, 6, 6, 0, 0, 45, 45);

	sim->runSimulation(argc, argv);
	sim->replyMessage();

	delete sim;
	return 0;
}
