#include <iostream>
#include <iomanip>
#include "imobotik.h"

int main(void) {
    CiMobotIK ik(2,1);

	ik.iMobotAnchor(ANCHOR_LE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	ik.iMobotAttach(1, 0, 6, 1, 0, 0, 0, 0);
	ik.addEffector(0, 1, 6);
	ik.setTargetPosition(0, 0.25, 0.3, 0.0);
    ik.setTargetRotation(0, 0, 0, 0);
    ik.computeInverseKinematics();

    cout << fixed << setprecision(3);
	cout << "Effector 0:" << endl;
	cout << "    X: " << setw(8) << ik.getEffectorX(0) << setw(8) << ik.getTargetX(0) << endl;
    cout << "    Y: " << setw(8) << ik.getEffectorY(0) << setw(8) << ik.getTargetY(0) << endl;
    cout << "    Z: " << setw(8) << ik.getEffectorZ(0) << setw(8) << ik.getTargetZ(0) << endl;
    cout << "  Psi: " << setw(8) << ik.getEffectorPsi(0) << setw(8) << ik.getTargetPsi(0) << endl;
    cout << "Theta: " << setw(8) << ik.getEffectorTheta(0) << setw(8) << ik.getTargetTheta(0) << endl;
    cout << "  Phi: " << setw(8) << ik.getEffectorPhi(0) << setw(8) << ik.getTargetPhi(0) << endl;

	return 0;
}