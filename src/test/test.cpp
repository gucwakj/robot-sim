#include <iostream>
#include "imobotsim.h"
#include "imobotik.h"
using namespace std;

int main(int argc, char *argv[]) {
    cout << "Inverse Kinematics...";
    CiMobotIK *ik = new CiMobotIK(2, 1);
    ik->iMobotAnchor(ANCHOR_LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    ik->iMobotAttach(1, 0, 6, 1, 0, 0, 0, 0);
    ik->addEffector(0, 1, 6);
    ik->setTargetPosition(0, 0.35, 0.15, 0.10);
    ik->setTargetRotation(0, 0, 0, 0);
    ik->runSimulation(argc, argv);
    switch (ik->getReplyMessage()) {
        case IK_SUCCESS:
            cout << " success!" << endl;
            break;
        case IK_ERROR_TIME:
            cout << " failed." << endl;
            return -1;
        case IK_ERROR_STALL:
            cout << " stalled." << endl;
            return -1;
    }

    dReal *ang = new dReal[ik->getNumAngles()*ik->getNumAngles()]();
    ik->formatAngles(2, ang);
    for ( int i = 0; i < 8; i++ ) {
        for ( int j = 0; j < 8; j++ ) {
            cout.precision(4);cout.width(8);
            cout << ang[i*8 + j] << "\t";
        }
        cout << endl;
    }*/

    cout << "Forward Dynamics...";
    CiMobotSim *sim = new CiMobotSim(2, 8, 1, 1, ang);
    sim->setTime(5);
    sim->setTarget(0, 0.35, -0.10, 0.15+0.2);
	sim->groundPlane(0, 0, 0, 1, 0);
    sim->iMobotAnchor(0, ENDCAP_L, 0, 0, 0.2-BODY_HEIGHT/2, 0, 0, 0, 0, 0, 0, 0);
    sim->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);
    sim->runSimulation(argc, argv);
    switch (sim->getReplyMessage()) {
        case SIM_SUCCESS:
            cout << " success!" << endl;
            break;
        case SIM_ERROR_TIME:
            cout << " failed." << endl;
            break;
        case IK_ERROR_STALL:
            cout << " stalled." << endl;
            break;
    }

    delete ang;
    delete ik;
	delete sim;
	return 0;
}
