#include <iostream>
#include "imobotik.h"

int main(int argc, char *argv[]) {
	CiMobotIK *ik = new CiMobotIK(2, 1);

	ik->iMobotAnchor(ANCHOR_LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	ik->iMobotAttach(1, 0, 6, 1, 0, 0, 0, 0);
	ik->addEffector(0, 1, 6);
	ik->setTargetPosition(0, 0.35, 0.2, 0.0);
    ik->setTargetRotation(0, 0, 0, 0);
	ik->runSimulation(argc, argv);

    //double *ang = new double[ik->getNumAngles()];
    //ik->getAngles(ang);
    //cout << ang[0] << "\t" << ang[1] << "\t" << ang[2] << "\t" << ang[3] << "\t" << ang[4] << "\t" << ang[5] << "\t" << ang[6] << "\t" << ang[7] << endl;
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