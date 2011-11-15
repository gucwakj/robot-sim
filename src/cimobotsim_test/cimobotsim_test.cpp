#include <iostream>
#include "imobotsim.h"
using namespace std;

int main(int argc, char *argv[]) {
    dReal ang[] = { 0,  45, 45, 0};
	CiMobotSim *sim = new CiMobotSim(2, 1, 1, ang);

	//sim->setCOR(0.45, 0.45);
	//sim->setMu(0.3, 0.1);
	//sim->setTime(1.0);
	//sim->setAngVel(vel);

	sim->groundPlane(0, 0, 0, 1, 0);

	sim->iMobotBuild(0, 0, 0, 0);
	//sim->iMobotBuild(0, 0, 0, 5, 15, 25, 45);
    //sim->iMobotBuild(0, 0, 0, 5, 0, 0, 0, 0, 45, 45, 0);
    sim->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);

	sim->runSimulation(argc, argv);
    cout << "Reply Message: " << sim->getReplyMessage() << endl;

	delete sim;
	return 0;
}
