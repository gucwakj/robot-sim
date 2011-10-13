#include <iostream>
#include "imobotik.h"

int main(int argc, char *argv[]) {
	CiMobotIK *ik = new CiMobotIK(2, 1);

	//ik->iMobotAnchor(0, 0, 0, 0, 0, 0, 0);
	ik->iMobotAnchor(LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	ik->iMobotAttach(1, 0, 4, 1, 0, 0, 10, 0);
	ik->addEffector(1, 1, 6);

	ik->setTarget(0, 3.0, 0.1, 0.4);

	ik->runSimulation(argc, argv);

	cout << "Effector 1: <" << ik->getEffectorX(1) << "," << ik->getEffectorY(1) << "," << ik->getEffectorZ(1) << ">" << endl;

	delete ik;
	return 0;
}