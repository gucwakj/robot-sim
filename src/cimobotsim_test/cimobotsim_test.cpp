#include "imobotsim.h"

dReal ang[] = {	0,	0,	25,	0,
				0,	0,	45,	0,
				0,	0,	65,	0,
				0,	0,	85,	0};
dReal vel[] = {	1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1};
int main(int argc, char* argv[]) {
	CiMobotSim *sim = new CiMobotSim(1, 4, 1, 2.0, 0.3, 0.1, 0.45, 0.45, ang, vel);

	sim->groundPlane(0, 0, 0, 1, 0);
	sim->iMobotBuild(0, 0, -4, 0);
	//sim->printData();
	sim->run(argc, argv);
	sim->replyMessage();

	delete sim;
	return 0;
}
