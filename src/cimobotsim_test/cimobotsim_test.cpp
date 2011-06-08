#include "imobotsim.h"

/* Test angles for simulation */
dReal ang[] = {	0,	0,	45,	0};
dReal vel[] = {	1,	0.3,	1,	1};

/* video replication */
/*dReal ang[] = {	0,	30,		0,		0, // forward
				0,	30,	-30,	0,
				0,	0,		-30,	0,
				0,	0,	0,	0,
				360,	0,	0,	-360,		// right
				0,	0,	-95.5293,	0,		// camera stand
				0,	-85,	123456789,	0,
				0,	-85,	123456789,	45,
				0,	-90,	10,		0,
				0,	-90,	10,		180};
dReal vel[] = {	0.1,	1,	1,	1,		// forward
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,		// right
				1,	1,	1,	1,		// camera stand
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	0.1,	1,
				1,	1,	1,	1};*/

/* inchworm forward */
/*dReal ang[] = {	0, -30, 0, 0,
				0, -30, -30, 0,
				0, 0, -30, 0,
				0, 0, 0, 0,
				0, -30, 0, 0,
				0, -30, -30, 0,
				0, 0, -30, 0,
				0, 0, 0, 0,
				0, -30, 0, 0,
				0, -30, -30, 0,
				0, 0, -30, 0,
				0, 0, 0, 0};
dReal vel[] = {	1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1,
				1,	1,	1,	1};
*/

/* // rolling
dReal ang[] = {270, 0, 0, -270};
dReal vel[] = {1, 1, 1, 1};*/

int main(int argc, char* argv[]) {
	CiMobotSim *sim = new CiMobotSim(1,1,1,1.0,ang);		/*test*/
	sim->setMu(0.3, 0.1);
	sim->setCOR(0.45, 0.45);
	//sim->setAngVel(vel);

	//dMatrix3 R;
	//dRSetIdentity(R);
	sim->groundPlane(0, 0, 0, 1, 0);
	//sim->iMobotBuild(0, 0, 0, 0);
	sim->iMobotBuildPositioned(0, 0, 0, 0, 0, 0, 0, 0, 45, 45, 0);
	sim->run(argc, argv);
	sim->replyMessage();

	delete sim;
	return 0;
}
