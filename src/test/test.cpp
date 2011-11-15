#include <iostream>
#include "imobotsim.h"
#include "imobotik.h"
using namespace std;

int main(int argc, char *argv[]) {
    cout << endl << "Inverse Kinematics" << endl;
    CiMobotIK *ik = new CiMobotIK(2, 1);
    ik->iMobotAnchor(ANCHOR_LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    ik->iMobotAttach(1, 0, 6, 1, 0, 0, 0, 0);
    ik->addEffector(0, 1, 6);
    ik->setTargetPosition(0, 0.35, 0.15, 0.0);
    ik->setTargetRotation(0, 0, 10, 0);
    ik->runSimulation(argc, argv);
    delete ik;

    dReal ang[] = { 0,  0,  0,  0,  0, 0, 45, 0};
    cout << endl << "Forward Dynamics" << endl;
    CiMobotSim *sim = new CiMobotSim(2, 1, 1, ang);
	sim->groundPlane(0, 0, 0, 1, 0);
    sim->iMobotAnchor(0, LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    sim->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);
	sim->runSimulation(argc, argv);
	delete sim;

	return 0;
}
