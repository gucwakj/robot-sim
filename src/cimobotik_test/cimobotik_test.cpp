#include <iostream>
#include "imobotik.h"

int main(int argc, char *argv[]) {
	CiMobotIK *ik = new CiMobotIK(3, 1);

	ik->iMobotAnchor(0, 0, 0, 0, 20, 0, 0);
	ik->iMobotAttach(1, 0, 1, 6, 0, 0, 0, 0);
	ik->iMobotAttach(2, 1, 1, 6, 0, 0, 0, 0);
	ik->addEffector(1, 2);

	ik->setTarget(0, 3.0, 0.1, 0.4);

	ik->runSimulation(argc, argv);

	cout << "Effector Z: " << ik->getEffectorZ(1) << endl;

	delete ik;
	return 0;
}