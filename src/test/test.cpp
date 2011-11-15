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

    dReal *ang = new dReal[ik->getNumAngles()*ik->getNumAngles()]();
    ik->formatAngles(1, ang);

    /*for ( int i = 0; i < 8; i++ ) {
        for ( int j = 0; j < 8; j++ ) {
            cout.precision(4);cout.width(8);
            cout << ang2[i*8 + j] << "\t";
        }
        cout << endl;
    }
    cout << endl;*/

    cout << endl << "Forward Dynamics" << endl;
    CiMobotSim *sim = new CiMobotSim(2, 8, 1, ang);
	sim->groundPlane(0, 0, 0, 1, 0);
    sim->iMobotAnchor(0, LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    sim->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);
    sim->runSimulation(argc, argv);

    delete ang;
    delete ik;
	delete sim;
	return 0;
}
