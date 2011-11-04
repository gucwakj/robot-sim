#include <iostream>
#include "imobotik.h"

int main(int argc, char *argv[]) {
	CiMobotIK *ik = new CiMobotIK(2, 1);

	//ik->iMobotAnchor(0, 0, 0, 0, 0, 0, 0);
	ik->iMobotAnchor(LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	ik->iMobotAttach(1, 0, 6, 1, 0, 0, 0, 0);
	ik->addEffector(0, 1, 6);

	ik->setTargetPosition(0, 0.25, 0.25, 0.0);
    ik->setTargetRotation(0, 0, 0, 10);

	ik->runSimulation(argc, argv);

	cout << "Effector 0:" << endl;
	cout << "    X: " << ik->getEffectorX(0) << "\t" << ik->getTargetX(0) << endl;
	cout << "    Y: " << ik->getEffectorY(0) << "\t" << ik->getTargetY(0) << endl;
	cout << "    Z: " << ik->getEffectorZ(0) << "\t" << ik->getTargetZ(0) << endl;
	cout << "  Psi: " << ik->getEffectorPsi(0) << "\t" << ik->getTargetPsi(0) << endl;
	cout << "Theta: " << ik->getEffectorTheta(0) << "\t" << ik->getTargetTheta(0) << endl;
	cout << "  Phi: " << ik->getEffectorPhi(0) << "\t" << ik->getTargetPhi(0) << endl;

	delete ik;
	return 0;
}