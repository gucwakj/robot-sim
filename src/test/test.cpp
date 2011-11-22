#include <iostream>
#include "imobotsim.h"
#include "imobotik.h"
using namespace std;

int main(int argc, char *argv[]) {
    cout << "Inverse Kinematics" << endl;
    CiMobotIK *ik = new CiMobotIK(2, 1);
    ik->iMobotAnchor(ANCHOR_LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    ik->iMobotAttach(1, 0, 6, 1, 0, 0, 0, 0);
    ik->addEffector(0, 1, 6);
    ik->setTargetPosition(0, 0.35, 0.15, 0.10);
    ik->setTargetRotation(0, 0, 0, 0);
    ik->runSimulation(argc, argv);
    switch (ik->getReplyMessage()) {
        case IK_ERROR_TIME:
            cout << "IK Failed to Converge" << endl;
            return -1;
        case IK_ERROR_STALL:
            cout << "IK Stall" << endl;
            return -1;
    }

    dReal *ang = new dReal[ik->getNumAngles()*ik->getNumAngles()]();
    ik->formatAngles(2, ang);
    //cout << "<" << ik->getTargetX(0) << ", " << ik->getTargetY(0) << ", " << ik->getTargetZ(0) << ">" << endl;
    //cout << "<" << ik->getEffectorX(0) << ", " << ik->getEffectorY(0) << ", " << ik->getEffectorZ(0) << ">" << endl;
    for ( int i = 0; i < 8; i++ ) {
        for ( int j = 0; j < 8; j++ ) {
            cout.precision(4);cout.width(8);
            cout << ang[i*8 + j] << "\t";
        }
        cout << endl;
    }
    cout << endl;
    /*cout << "Effector 0:" << endl;
    cout << "    X: " << ik->getEffectorX(0) << "\t" << ik->getTargetX(0) << endl;
    cout << "    Y: " << ik->getEffectorY(0) << "\t" << ik->getTargetY(0) << endl;
    cout << "    Z: " << ik->getEffectorZ(0) << "\t" << ik->getTargetZ(0) << endl;
    cout << "  Psi: " << ik->getEffectorPsi(0) << "\t" << ik->getTargetPsi(0) << endl;
    cout << "Theta: " << ik->getEffectorTheta(0) << "\t" << ik->getTargetTheta(0) << endl;
    cout << "  Phi: " << ik->getEffectorPhi(0) << "\t" << ik->getTargetPhi(0) << endl;*/
    /*dReal ang2[64] = {0};
    ang2[1] = -20;
    for ( int i = 0; i < 8; i++ ) {
        for ( int j = 0; j < 8; j++ ) {
            cout.precision(4);cout.width(8);
            cout << ang2[i*8 + j] << "\t";
        }
        cout << endl;
    }
    cout << endl;*/

    cout << "Forward Dynamics" << endl;
    CiMobotSim *sim = new CiMobotSim(2, 8, 1, 1, ang);
    sim->setTime(5);
    sim->setTarget(0, -0.1, 0.0, 0.10);
	sim->groundPlane(0, 0, 0, 1, 0);
    sim->iMobotAnchor(0, LE, 0, 0, 0.1, 0, 90, 0, 0, 0, 0, 0);
    sim->iMobotBuildAttached(1, 0, 6, 1, 0, 0, 0, 0);
    sim->runSimulation(argc, argv);
    cout << "Reply Message: " << sim->getReplyMessage() << endl;

    //delete ang;
    //delete ik;
	delete sim;
	return 0;
}
