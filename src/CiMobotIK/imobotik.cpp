#include <cmath>
#include <iostream>
#include "imobotik.h"

using namespace std;

CiMobotIK::CiMobotIK(int num_bot, int num_targets) {
	this->m_num_bot = num_bot;
	this->m_num_targets = num_targets;
	this->m_t_step = 0.004;
	this->m_t = 0.0;
	this->m_del_theta = new bool[NUM_DOF*num_bot + num_targets];
	this->node = new Node * [NUM_DOF*num_bot];
	this->node_right = new Node * [NUM_DOF*num_bot];
	this->node_effector = new Node * [num_targets];
	this->target = new VectorR3[num_targets];
}

CiMobotIK::~CiMobotIK(void) {
	delete this->jacob;
	delete [] this->target;
	delete [] this->m_del_theta;
	for ( int i = NUM_DOF*this->m_num_bot + this->m_num_targets - 1; i >= 0; i-- ) {
		delete this->node[i];
	}
	delete [] this->node;
}

void CiMobotIK::iMobotAnchor(double x, double y, double z, double r_le, double r_lb, double r_rb, double r_re) {
	this->node[0] = new Node(VectorR3(x, y + END_DEPTH, z), VectorR3(0, 1, 0), JOINT, D2R(-180), D2R(180), D2R(r_le));
	this->tree.insertRoot(node[0]);
	this->node[1] = new Node(this->node[0]->getSInit() + VectorR3(0, BODY_END_DEPTH + BODY_LENGTH, 0), VectorR3(0, 0, 1), JOINT, D2R(-180), D2R(180), D2R(r_lb));
	this->tree.insertLeftChild(node[0], node[1]);
	this->node[2] = new Node(this->node[1]->getSInit() + VectorR3(0, CENTER_LENGTH, 0), VectorR3(0, 0, 1), JOINT, D2R(-180), D2R(180), D2R(r_rb));
	this->tree.insertLeftChild(node[1], node[2]);
	this->node[3] = new Node(this->node[2]->getSInit() + VectorR3(0, BODY_END_DEPTH + BODY_LENGTH, 0), VectorR3(0, 1, 0), JOINT, D2R(-180), D2R(180), D2R(r_re));
	this->tree.insertLeftChild(node[2], node[3]);
}
void CiMobotIK::iMobotAnchor(int end, double x, double y, double z, double psi, double theta, double phi, double r_le, double r_lb, double r_rb, double r_re) {
	double R[9];
	rotation_matrix_from_euler_angles(R, D2R(psi), D2R(theta), D2R(phi));
	this->m_psi = D2R(psi);
	this->m_theta = D2R(theta);
	this->m_phi = D2R(phi);

	if ( end == LE ) {
		double le = END_DEPTH;
		double lb = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double rb = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double re = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
		this->node[0] = new Node(VectorR3(R[0]*le + x, R[3]*le + y, R[6]*le + z), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180), D2R(180), D2R(r_le));
		this->node[1] = new Node(VectorR3(R[0]*lb + x, R[3]*lb + y, R[6]*lb + z), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180), D2R(180), D2R(r_lb));
		this->node[2] = new Node(VectorR3(R[0]*rb + x, R[3]*rb + y, R[6]*rb + z), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180), D2R(180), D2R(r_rb));
		this->node[3] = new Node(VectorR3(R[0]*re + x, R[3]*re + y, R[6]*re + z), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180), D2R(180), D2R(r_re));
		this->tree.insertRoot(node[0]);
		this->tree.insertLeftChild(node[0], node[1]);
		this->tree.insertLeftChild(node[1], node[2]);
		this->tree.insertLeftChild(node[2], node[3]);
	}
	else if ( end == RE ) {
		double re = END_DEPTH;
		double rb = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double lb = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double le = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
		this->node[0] = new Node(VectorR3(R[0]*le + x, R[3]*le + y, R[6]*le + z), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180), D2R(180), D2R(r_le));
		this->node[1] = new Node(VectorR3(R[0]*lb + x, R[3]*lb + y, R[6]*lb + z), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180), D2R(180), D2R(r_lb));
		this->node[2] = new Node(VectorR3(R[0]*rb + x, R[3]*rb + y, R[6]*rb + z), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180), D2R(180), D2R(r_rb));
		this->node[3] = new Node(VectorR3(R[0]*re + x, R[3]*re + y, R[6]*re + z), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180), D2R(180), D2R(r_re));
		this->tree.insertRoot(node[3]);
		this->tree.insertLeftChild(node[3], node[2]);
		this->tree.insertLeftChild(node[2], node[1]);
		this->tree.insertLeftChild(node[1], node[0]);
	}

}

void CiMobotIK::iMobotAttach(int bot_num, int att_num, int face1, int face2, double r_le, double r_lb, double r_rb, double r_re) {
	double R[9];
	rotation_matrix_from_euler_angles(R, this->m_psi, this->m_theta, this->m_phi);

	if ( face1 == 4 && face2 == 1 ) {
		double le = BODY_WIDTH/2 + END_DEPTH;
		double lb = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double rb = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double re = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
		this->node_right[att_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit(), this->node[att_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[att_num*NUM_DOF + 2]->getThetaMin(), this->node[att_num*NUM_DOF + 2]->getThetaMax(), this->node[att_num*NUM_DOF + 2]->getThetaInit());
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*le, R[5]*le, R[8]*le), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*lb, R[5]*lb, R[8]*lb), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*rb, R[5]*rb, R[8]*rb), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*re, R[5]*re, R[8]*re), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertRightSibling(this->node[att_num*NUM_DOF + 2], this->node_right[att_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 0]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 0], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
	}
	else if ( face1 == 6 && face2 == 1 ) {
		double le = 2*END_DEPTH;
		double lb = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double rb = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double re = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*le, R[3]*le, R[6]*le), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*lb, R[3]*lb, R[6]*lb), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*rb, R[3]*rb, R[6]*rb), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*re, R[3]*re, R[6]*re), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 0]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 0], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
	}
	else if ( face1 == 6 && face2 == 2 ) {
		//double le = END_DEPTH;
		double lb[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER};
		double rb[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + CENTER_LENGTH};
		double re[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH};
		//this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*le, R[3]*le, R[6]*le), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*lb[0] + R[2]*lb[2], R[3]*lb[0] + R[5]*lb[2], R[6]*lb[0] + R[8]*lb[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*re[0] + R[2]*re[2], R[3]*re[0] + R[5]*re[2], R[6]*re[0] + R[8]*re[2]), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
	}
	else if ( face1 == 6 && face2 == 3 ) {
		//double le = END_DEPTH;
		double lb[3] = {END_DEPTH + BODY_WIDTH/2, 0, -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER};
		double rb[3] = {END_DEPTH + BODY_WIDTH/2, 0, -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - CENTER_LENGTH};
		double re[3] = {END_DEPTH + BODY_WIDTH/2, 0, -BODY_END_DEPTH - BODY_LENGTH + BODY_MOUNT_CENTER - CENTER_LENGTH - BODY_END_DEPTH - BODY_LENGTH};
		//this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*le, R[3]*le, R[6]*le), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*lb[0] + R[2]*lb[2], R[3]*lb[0] + R[5]*lb[2], R[6]*lb[0] + R[8]*lb[2]), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*re[0] + R[2]*re[2], R[3]*re[0] + R[5]*re[2], R[6]*re[0] + R[8]*re[2]), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
	}
	else if ( face1 == 6 && face2 == 4 ) {
		double le[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH};
		double lb[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + CENTER_LENGTH};
		double rb[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER};
		//double re = END_DEPTH;
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*le[0] + R[2]*le[2], R[3]*le[0] + R[5]*le[2], R[6]*le[0] + R[8]*le[2]), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*lb[0] + R[2]*lb[2], R[3]*lb[0] + R[5]*lb[2], R[6]*lb[0] + R[8]*lb[2]), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		//this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*re[0] + R[2]*re[2], R[3]*re[0] + R[5]*re[2], R[6]*re[0] + R[8]*re[2]), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 0]);
	}
	else if ( face1 == 6 && face2 == 5 ) {
		double le[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH};
		double lb[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER + CENTER_LENGTH};
		double rb[3] = {END_DEPTH + BODY_WIDTH/2, 0, BODY_END_DEPTH + BODY_LENGTH - BODY_MOUNT_CENTER};
		//double re = END_DEPTH;
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*le[0] + R[2]*le[2], R[3]*le[0] + R[5]*le[2], R[6]*le[0] + R[8]*le[2]), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*lb[0] + R[2]*lb[2], R[3]*lb[0] + R[5]*lb[2], R[6]*lb[0] + R[8]*lb[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		//this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*re[0] + R[2]*re[2], R[3]*re[0] + R[5]*re[2], R[6]*re[0] + R[8]*re[2]), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 0]);
	}
	else if ( face1 == 6 && face2 == 6 ) {
		double le = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
		double lb = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double rb = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double re = 2*END_DEPTH;
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*le, R[3]*le, R[6]*le), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*lb, R[3]*lb, R[6]*lb), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*rb, R[3]*rb, R[6]*rb), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*re, R[3]*re, R[6]*re), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 3]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 0]);
	}
}

void CiMobotIK::addEffector(int eff_num, int bot_num, int face) {
	double R[9];
	rotation_matrix_from_euler_angles(R, this->m_psi, this->m_theta, this->m_phi);
	//VectorR3 eff = END_DEPTH*this->node[bot_num*NUM_DOF + offset]->getWInit();
	double eff = END_DEPTH;
	int offset = 0;

	if ( face == 1 )
		offset = 0;
	else if ( face == 6 )
		offset = 3;

	this->node_effector[eff_num] = new Node(this->node[bot_num*NUM_DOF + offset]->getSInit() + VectorR3(R[0]*eff, R[3]*eff, R[6]*eff), VectorR3(0.0, 0.0, 0.0), EFFECTOR);
	this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + offset], this->node_effector[eff_num]);
}

void CiMobotIK::setCurrentMode(int mode) {
	this->jacob->setCurrentMode(mode);
}

void CiMobotIK::setCurrentType(int type) {
	this->jacob->setCurrentType(type);
}

void CiMobotIK::setCurrentDLSMode(int mode) {
	this->jacob->setCurrentDLSMode(mode);
}

void CiMobotIK::setDampingDLS(double lambda) {
	this->jacob->setDampingDLS(lambda);
}

void CiMobotIK::setTarget(int num, double x, double y, double z) {
	this->target[num].Set(x, y, z);
}

int CiMobotIK::getCurrentMode(void) {
	return this->jacob->getCurrentMode();
}

int CiMobotIK::getCurrentType(void) {
	return this->jacob->getCurrentType();
}

int CiMobotIK::getCurrentDLSMode(void) {
	return this->jacob->getCurrentDLSMode();
}

void CiMobotIK::getEffector(int num, double &x, double &y, double &z) {
	const VectorR3 &n = this->node_effector[num]->getS();
	x = n.x;
	y = n.y;
	z = n.z;
}

double CiMobotIK::getEffectorX(int num) {
	const VectorR3 &n = this->node_effector[num]->getS();
	return n.x;
}

double CiMobotIK::getEffectorY(int num) {
	const VectorR3 &n = this->node_effector[num]->getS();
	return n.y;
}

double CiMobotIK::getEffectorZ(int num) {
	const VectorR3 &n = this->node_effector[num]->getS();
	return n.z;
}

void CiMobotIK::getTarget(int num, double &x, double &y, double &z) {
	x = target[num].x;
	y = target[num].y;
	z = target[num].z;
}

double CiMobotIK::getTargetX(int num) {
	return this->target[num].x;
}

double CiMobotIK::getTargetY(int num) {
	return this->target[num].y;
}

double CiMobotIK::getTargetZ(int num) {
	return this->target[num].z;
}

void CiMobotIK::runSimulation(int argc, char **argv) {
	bool loop = true;
	this->jacob = new Jacobian(&(this->tree), this->target);
	this->tree.init();
	this->tree.compute();

	while( loop ) {
		this->jacob->computeJacobian();			// set up Jacobian and deltaS vectors
		this->jacob->calcDeltaThetas();			// calculate delta Theta values
		this->jacob->updateThetas();			// apply the change in the theta values
		this->jacob->updatedSClampValue();		// update distance to target position
		//this->jacob->updateErrorArray();		// update error of effector to target position

		this->print_intermediate_data();

		this->update_targets();					// update target to new values
		this->set_flags();						// set flags for completion
		this->increment_step();					// increment time step
		loop = this->end_simulation();			// check to end simulation
	}
}

void CiMobotIK::print_intermediate_data(void) {
	cout << this->m_t_count << "\t" << this->m_t << "\t";
	for ( int i = 0; i < this->m_num_bot*NUM_DOF+this->m_num_targets; i++ ) {
		if ( this->node[i] ) {
			//cout << "Node " << i << endl;
			//cout << "     S: " << this->node[i]->getS() << endl;
			//cout << "S Init: " << this->node[i]->getSInit() << endl;
			//cout << "     W: " << this->node[i]->getW() << endl;
			//cout << "W Init: " << this->node[i]->getWInit() << endl;
			//cout << " Theta: " << this->node[i]->getTheta() << endl;
			cout << this->m_del_theta[i] << " ";
			//cout << this->node[i]->getTheta() << "\t";
		}
	}
	cout << endl;
	//cout << endl;
}

void CiMobotIK::update_targets(void) {
	//this->target[0].Set(3.f, 0.1f, 0.3f);
}

void CiMobotIK::increment_step(void) {
	this->m_t += this->m_t_step;
	this->m_t_count++;
}

bool CiMobotIK::end_simulation(void) {
	if ( this->m_t_count == 200 ) return false;
	if ( is_true(this->m_del_theta, NUM_DOF*this->m_num_bot) ) return false;
	return true;
}

void CiMobotIK::set_flags(void) {
	Node *n = this->tree.getRoot();
	int i = 0;
	while ( n ) {
		this->m_del_theta[i] = 1;
		if ( n->isJoint() && (fabs(this->jacob->getDeltaTheta(n->getJointNum())) > 0.00004) )
			this->m_del_theta[i] = 0;
		i++;
		n = this->tree.getSuccessor(n);
	}
}

void CiMobotIK::rotation_matrix_from_euler_angles(double *R, double psi, double theta, double phi) {
	double	sphi = sin(phi), 	cphi = cos(phi),
			stheta = sin(theta),ctheta = cos(theta),
			spsi = sin(psi),	cpsi = cos(psi);

	R[0] =  cphi*ctheta;
	R[1] = -cphi*stheta*cpsi + sphi*spsi;
	R[2] =  cphi*stheta*spsi + sphi*cpsi;
	R[3] =  stheta;
	R[4] =  ctheta*cpsi;
	R[5] = -ctheta*spsi;
	R[6] = -sphi*ctheta;
	R[7] =  sphi*stheta*cpsi + cphi*spsi;
	R[8] = -sphi*stheta*spsi + cphi*cpsi;
}

bool CiMobotIK::is_true(bool *a, int length) {
	for ( int i = 0; i < length; i++ ) {
		if ( a[i] == false )
			return false;
	}
	return true;
}

inline double CiMobotIK::D2R(double deg) {
	return deg*M_PI/180;
}

inline double CiMobotIK::R2D(double rad) {
	return rad/M_PI*180;
}
