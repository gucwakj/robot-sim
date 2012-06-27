#include <iostream>
#include "mobotfd.h"
using namespace std;

int main(int argc, char *argv[]) {
	CMobotFD fd(2, 1);

    double ang[] = { 0, 45, 45, -45, 45, 45, 45, 0};
    fd.setAngles(ang);

	fd.iMobotBuild(0, 0, 0, 0, 0, 0, 0);
    fd.iMobotBuildAttached(1, 0, 6, 1, 0, 45, 45, 0);

	fd.runSimulation(argc, argv);
    cout << "Reply Message: " << fd.getReplyMessage() << endl;

	return 0;
}
