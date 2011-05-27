#include "imobotsim.h"

/* Test angles for simulation */
/*dReal ang[] = {	0,	0,	-25,	0,
		0,	-25,	123456789,	0,
		0,	0,	0,	0,
		0,	0,	0,	0};
dReal vel[] = {	1,	1,	1,	1,
		1,	1,	1,	1,
		1,	1,	1,	1,
		1,	1,	1,	1};
*/
/* video replication */
dReal ang[] = {	0,	-60,		0,		0, // forward
				0,	-60,	-60,	0,
				0,	0,		-60,	0,
				0,	0,	0,	0,
				720,	0,	0,	-720,		// right
				0,	0,	-95.5293,	0,		// camera stand
				0,	0,	-95.5293,	45,
				0,	-85,	123456789,	0,
				0,	-90,	10,		0,
				0,	-90,	10,		45};
dReal vel[] = {	1,	1,	1,	1,		// forward
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,		// right
				1,	1,	1,	1,		// camera stand
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	0.1,	1,
				1,	1,	1,	1};

int main(int argc, char* argv[]) {
	/*CiMobotSim *sim = new CiMobotSim(1, 4, 1, 5.0, 0.3, 0.1, 0.45, 0.45, ang, vel);*/ /*test*/
	CiMobotSim *sim = new CiMobotSim(1,10,1,10.0,ang,vel);		/*video*/
	sim->setMu(0.3, 0.1);
	sim->setCOR(0.45, 0.45);

	sim->groundPlane(0, 0, 0, 1, 0);
	sim->iMobotBuild(0, 0, 0, 0);
	sim->run(argc, argv);
	sim->replyMessage();

	delete sim;
	return 0;
}
