#include <iostream>
#include "imobotfd.h"
using namespace std;

int main(int argc, char *argv[]) {
	CiMobotFD *fd = new CiMobotFD(2, 1);

    dReal ang[] = { 0, 45, 45, -45, 45, 45, 45, 0};
    fd->setAngles(ang);

	fd->iMobotBuild(0, 0, 0, 0, 0, 0, 0);
    fd->iMobotBuildAttached(1, 0, 6, 1, 0, 45, 45, 0);

	fd->runSimulation(argc, argv);
    cout << "Reply Message: " << fd->getReplyMessage() << endl;

	delete fd;
	return 0;
}