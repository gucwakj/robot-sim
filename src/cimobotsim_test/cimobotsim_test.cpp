#include "imobotsim.h"

/* Test angles for simulation */
dReal ang[] = {	0,	45,	45,	45,
				0,	45,	45,	-45};
dReal vel[] = {	1,	0.3,	1,	1};

int main(int argc, char* argv[]) {
	CiMobotSim *sim = new CiMobotSim(1, 2, 1, 1.0, ang);
	sim->setMu(0.3, 0.1);
	sim->setCOR(0.45, 0.45);
	//sim->setAngVel(vel);

	dMatrix3 R;
	dRSetIdentity(R);
	sim->groundPlane(0, 0, 0, 1, 0);
	//sim->iMobotBuild(0, 0, 0, 0);
	sim->iMobotBuildPositioned(0, 0, 0, 0, R, 45, 45, 45, 45);
	sim->run(argc, argv);
	sim->replyMessage();

	delete sim;
	return 0;
}
