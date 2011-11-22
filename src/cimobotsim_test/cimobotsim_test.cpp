#include <iostream>
#include "imobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
    dReal ang[] = { 0,  45, -45, 0, 0, 0, 0, 0,
                    0,  45, -35, 0, 0, -10, 0, 0,
                    0,  45, -25, 0, 0, -20, 0, 0,
                    0,  45, -15, 0, 0, -30, 0, 0};
	CiMobotSim *sim = new CiMobotSim(2, 4, 1, 1, ang);

	//sim->setCOR(0.45, 0.45);
	//sim->setMu(0.3, 0.1);
	//sim->setTime(3);
    //sim->setAngVel(vel);
    sim->setTarget(0, -0.1, 0.0, 0.10);

	sim->groundPlane(0, 0, 0, 1, 0);

	sim->iMobotBuild(0, 0, 0, 0.1, 0, 90, 0);
    sim->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);
    //sim->iMobotAnchor(0, LE, 0, 0, 0.1, 0, 90, 0, 0, 0, 0, 0);
    //sim->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);

	sim->runSimulation(argc, argv);
    cout << "Reply Message: " << sim->getReplyMessage() << endl;

	delete sim;
	return 0;
}
