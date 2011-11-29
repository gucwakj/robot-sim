#include <iostream>
#include "imobotfd.h"
using namespace std;

int main(int argc, char *argv[]) {
    /*dReal ang[] = { 0,  45, -45, 0, 0, 0, 0, 0,
                    0,  45, -35, 0, 0, -10, 0, 0,
                    0,  45, -25, 0, 0, -20, 0, 0,
                    0,  45, -15, 0, 0, -30, 0, 0};*/
	CiMobotFD *fd = new CiMobotFD(2, 1, 1, 1);

    dReal ang[] = { 0, 45, 45, 0, 0, 0, 0, 0};
    fd->setAngles(ang);
    dReal vel[] = { 1, 1, 0.4, 0.5, 1, 1, 1, 1};
    fd->setAngularVelocity(vel);
	//fd->setCOR(0.45, 0.45);
	//fd->setMu(0.3, 0.1);
    //fd->setTime(3);
    fd->setTarget(0, -0.1, 0.0, 0.10);

	//fd->groundPlane(0, 0, 0, 1, 0);

	//fd->iMobotBuild(0, 0, 0, 0.1, 0, 90, 0);
    //fd->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);
    //fd->iMobotAnchor(0, LE, 0, 0, 0.1, 0, 90, 0, 0, 0, 0, 0);
    //fd->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);

	//fd->runSimulation(argc, argv);
    cout << "Reply Message: " << fd->getReplyMessage() << endl;

	delete fd;
	return 0;
}
