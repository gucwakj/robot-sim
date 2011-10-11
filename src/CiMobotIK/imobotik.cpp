#include <cmath>
#include <iostream>
#include "imobotik.h"

using namespace std;

#ifdef ENABLE_GRAPHICS
CiMobotIK *g_imobotik;
#endif

CiMobotIK::CiMobotIK(int num_bot, int num_targets) {
	this->m_num_bot = num_bot;
	this->m_num_targets = num_targets;
	this->m_t_step = 0.004;
	this->m_t = 0.0;
	this->node = new Node * [NUM_DOF*num_bot + num_targets];
	this->target = new VectorR3[num_targets];

	#ifdef ENABLE_GRAPHICS
	RotMatrix[0][1] = RotMatrix[0][2] = RotMatrix[0][3] = 0.;
	RotMatrix[1][0]                   = RotMatrix[1][2] = RotMatrix[1][3] = 0.;
	RotMatrix[2][0] = RotMatrix[2][1]                   = RotMatrix[2][3] = 0.;
	RotMatrix[3][0] = RotMatrix[3][1] = RotMatrix[3][3]                   = 0.;
	RotMatrix[0][0] = RotMatrix[1][1] = RotMatrix[2][2] = RotMatrix[3][3] = 1.;
	ANGFACT = 1.0;
	SCLFACT = 0.005f;
	MINSCALE = 0.05f;
	LEFT   = 4;
	MIDDLE = 2;
	RIGHT  = 1;
	ORTHO = 1;
	PERSP = 0;
	ActiveButton = 1;
	BACKCOLOR[0] = 0.0;
	BACKCOLOR[1] = 0.0;
	BACKCOLOR[2] = 0.0;
	BACKCOLOR[3] = 0.0;
	AXES_COLOR[0] = 1.0;
	AXES_COLOR[1] = 0.5;
	AXES_COLOR[2] = 0.0;
	AXES_WIDTH = 3.0;
	g_imobotik = this;
	#endif
}

CiMobotIK::~CiMobotIK(void) {
	//delete this->jacob;
	//delete this->target;
	//delete this->tree;
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

/*void CiMobotIK::iMobotAttach(int bot_num, int att_num, int face1, int face2, double r_le, double r_lb, double r_rb, double r_re) {
	this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(0, 2*END_DEPTH, 0), VectorR3(0, 1, 0), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
	this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 0]);
	this->node[bot_num*NUM_DOF + 1] = new Node(this->node[bot_num*NUM_DOF + 0]->getSInit() + VectorR3(0, BODY_END_DEPTH + BODY_LENGTH, 0), VectorR3(0, 0, 1), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
	this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 0], this->node[bot_num*NUM_DOF + 1]);
	this->node[bot_num*NUM_DOF + 2] = new Node(this->node[bot_num*NUM_DOF + 1]->getSInit() + VectorR3(0, CENTER_LENGTH, 0), VectorR3(0, 0, 1), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
	this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
	this->node[bot_num*NUM_DOF + 3] = new Node(this->node[bot_num*NUM_DOF + 2]->getSInit() + VectorR3(0, BODY_END_DEPTH + BODY_LENGTH, 0), VectorR3(0, 1, 0), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
	this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
}*/
void CiMobotIK::iMobotAttach(int bot_num, int att_num, int face1, int face2, double r_le, double r_lb, double r_rb, double r_re) {
	double R[9];
	rotation_matrix_from_euler_angles(R, this->m_psi, this->m_theta, this->m_phi);

	if ( face1 == 6 && face2 == 1 ) {
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

	this->node[this->m_num_bot*NUM_DOF + eff_num - 1] = new Node(this->node[bot_num*NUM_DOF + offset]->getSInit() + VectorR3(R[0]*eff, R[3]*eff, R[6]*eff), VectorR3(0.0, 0.0, 0.0), EFFECTOR);
	this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + offset], this->node[this->m_num_bot*NUM_DOF + eff_num - 1]);
}

#ifdef ENABLE_GRAPHICS
#define this g_imobotik
void CiMobotIK::runSimulation(int argc, char **argv) {
	this->jacob = new Jacobian(&(this->tree), this->target);
	this->tree.Init();
	this->tree.Compute();
	glutInit( &argc, argv);
	InitGraphics();
	ResetGraphics();
	glutMainLoop();
}
//#undef this
#else
void CiMobotIK::runSimulation(int argc, char **argv) {
	bool loop = false;
	this->jacob = new Jacobian(&(this->tree), this->target);
	this->tree.init();
	this->tree.compute();
	this->print_intermediate_data();

	while( loop ) {
		this->jacob->computeJacobian();			// set up Jacobian and deltaS vectors
		this->jacob->calcDeltaThetas();			// calculate delta Theta values
		this->jacob->updateThetas();			// apply the change in the theta values
		this->jacob->updatedSClampValue();		// update distance to target position
		//this->jacob->updateErrorArray();		// update error of effector to target position

		this->print_intermediate_data();

		this->increment_step();					// increment time step
		this->update_targets();					// update target to new values
		loop = this->end_simulation();			// check to end simulation
		this->m_t_count++;
	}
}
#endif

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
	const VectorR3 &n = this->node[num]->getS();
	x = n.x;
	y = n.y;
	z = n.z;
}

double CiMobotIK::getEffectorX(int num) {
	const VectorR3 &n = this->node[this->m_num_bot*NUM_DOF + num - 1]->getS();
	return n.x;
}

double CiMobotIK::getEffectorY(int num) {
	const VectorR3 &n = this->node[this->m_num_bot*NUM_DOF + num - 1]->getS();
	return n.y;
}

double CiMobotIK::getEffectorZ(int num) {
	const VectorR3 &n = this->node[this->m_num_bot*NUM_DOF + num - 1]->getS();
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

void CiMobotIK::print_intermediate_data(void) {
	//cout << this->m_t << "\t";
	for ( int i = 0; i < this->m_num_bot*NUM_DOF; i++ ) {
	//for ( int i = 4; i < 8; i++ ) {
		if ( this->node[i] ) {
			cout << "Node " << i << endl;
			cout << "     S: " << this->node[i]->getS() << endl;
			cout << "S Init: " << this->node[i]->getSInit() << endl;
			cout << "     W: " << this->node[i]->getW() << endl;
			cout << "W Init: " << this->node[i]->getWInit() << endl;
			cout << " Theta: " << this->node[i]->getTheta() << endl;
		}
	}
	cout << endl;
	cout << endl;
}

void CiMobotIK::update_targets(void) {
	//this->target[0].Set(3.f, 0.1f, 0.3f);
}

void CiMobotIK::increment_step(void) {
	this->m_t += this->m_t_step;
}

bool CiMobotIK::end_simulation(void) {
	if ( this->m_t_count == 10 ) {
		return false;
	}
	return true;
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

inline double CiMobotIK::D2R(double deg) {
	return deg*M_PI/180;
}

inline double CiMobotIK::R2D(double rad) {
	return rad/M_PI*180;
}

#ifdef ENABLE_GRAPHICS
void CiMobotIK::Animate(void) {
	glutSetWindow( GrWindow );
	glutPostRedisplay();
}

void CiMobotIK::ResetGraphics(void) {
	Scale  = 1.0;
	Scale2 = 0.0;		// because add 1. to it in Display()
	WhichProjection = ORTHO;
	Xrot = Yrot = 0.;
	TransXYZ[0] = TransXYZ[1] = TransXYZ[2] = 0.;

	RotMatrix[0][1] = RotMatrix[0][2] = RotMatrix[0][3] = 0.;
	RotMatrix[1][0]                   = RotMatrix[1][2] = RotMatrix[1][3] = 0.;
	RotMatrix[2][0] = RotMatrix[2][1]                   = RotMatrix[2][3] = 0.;
	RotMatrix[3][0] = RotMatrix[3][1] = RotMatrix[3][3]                   = 0.;
	RotMatrix[0][0] = RotMatrix[1][1] = RotMatrix[2][2] = RotMatrix[3][3] = 1.;

	glutSetWindow(GrWindow);
	glutPostRedisplay();
}

void CiMobotIK::Display(void) {
	do_update_step();
	float scale2;		/* real glui scale factor		*/

	glutSetWindow( GrWindow );
	glDrawBuffer( GL_BACK );
	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glEnable( GL_DEPTH_TEST );

	glShadeModel( GL_FLAT );
	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();

	if (WhichProjection == ORTHO)
		glOrtho(-3., 3.,     -1.7, 1.7,     0.1, 1000.);
	else
		gluPerspective(75., 1.,	0.1, 1000.);

	glMatrixMode( GL_MODELVIEW );
	glLoadIdentity();
	gluLookAt( -3., 0., 3.,     0., 0., 0.,     0., 1., 0. );
	glTranslatef( TransXYZ[0], TransXYZ[1], -TransXYZ[2] );

	glRotatef( Yrot, 0., 1., 0. );
	glRotatef( Xrot, 1., 0., 0. );
	glMultMatrixf( (const GLfloat *) RotMatrix );

	glScalef( Scale, Scale, Scale );
	scale2 = 1. + Scale2;		/* because glui translation starts at 0. */
	if( scale2 < MINSCALE )
		scale2 = MINSCALE;
	glScalef( scale2, scale2, scale2 );

	GLUI_Master.set_glutIdleFunc( (void (*)(void))&CiMobotIK::Animate );
	GLfloat target_ambient_and_diffuse[] = { 1.0f, 0.0f, 0.0f, 1.0f };
	GLfloat mat_ambient_and_diffuse[] = { 0.2f, 0.2f, 0.8f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, target_ambient_and_diffuse);

	// draw targets
	//VectorR3 target;
	for ( int i = 0; i < tree.GetNumEffector(); i++ ) {
		glPushMatrix();
		//target = this->jacob->GetTarget(i);
		//glTranslatef(target2[i].x, target2[i].y, target2[i].z);
		glutSolidSphere(0.1, 5, 5);
		glPopMatrix();
	}

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_ambient_and_diffuse);

	tree.Draw();

	glFlush();
	glutSwapBuffers();
}

void CiMobotIK::InitGraphics(void) {
	glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH );
	glutInitWindowSize(500, 300);
	glutInitWindowPosition(300, 200);

	GrWindow = glutCreateWindow("Title");
	glutSetWindowTitle("Title2");

	glClearColor( BACKCOLOR[0], BACKCOLOR[1], BACKCOLOR[2], BACKCOLOR[3] );
	glutSetWindow(GrWindow);
	glutDisplayFunc((void (*)(void))&CiMobotIK::Display);
	glutMouseFunc((void (*)(int, int, int, int))&CiMobotIK::MouseButton);
	glutMotionFunc((void (*)(int, int))&CiMobotIK::MouseMotion);

	GLfloat global_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat light0_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat light0_diffuse[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	GLfloat light0_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light0_position[] = { 3.0f, 3.0f, 3.0f, 0.0 };
	GLfloat mat_ambient_and_diffuse[] = { 0.2f, 0.2f, 0.8f, 1.0f };
	GLfloat mat_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat mat_shininess[] = { 15.0f };

	// light model
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_FALSE);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);

	// light0
	glLightfv(GL_LIGHT0, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light0_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light0_position);

	// light1
	glLightfv(GL_LIGHT1, GL_AMBIENT, light0_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light0_specular);
	glLightfv(GL_LIGHT1, GL_POSITION, light0_position);

	// material properties
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_ambient_and_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, mat_shininess);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
}

void CiMobotIK::MouseMotion( int x, int y ) {
	int dx, dy;     /* change in mouse coordinates      */

	dx = x - Xmouse;        /* change in mouse coords   */
	dy = y - Ymouse;

	/*if( ActiveButton & LEFT ) {
		switch( LeftButton ) {
			case ROTATE:
				Xrot += ( ANGFACT*dy );
				Yrot += ( ANGFACT*dx );
				break;
			case SCALE:
				Scale += SCLFACT * (float) ( dx - dy );
				if( Scale < MINSCALE )
					Scale = MINSCALE;
				break;
		}
	}*/

	if( ActiveButton & MIDDLE ) {
		Scale += SCLFACT * (float) ( dx - dy );
		if( Scale < MINSCALE )
			Scale = MINSCALE;
	}

	Xmouse = x;         /* new current position     */
	Ymouse = y;

	glutSetWindow( GrWindow );
	glutPostRedisplay();
}

void CiMobotIK::MouseButton( int button, int state, int x, int y ) {
	int b;          /* LEFT, MIDDLE, or RIGHT       */

	switch( button ) {
		case GLUT_LEFT_BUTTON:
			b = LEFT;       break;
		case GLUT_MIDDLE_BUTTON:
			b = MIDDLE;     break;
		case GLUT_RIGHT_BUTTON:
			b = RIGHT;      break;
		default:
			b = 0;
			cerr << "Unknown mouse button: " << button << "\n";
	}

	if( state == GLUT_DOWN ) {
		Xmouse = x;
		Ymouse = y;
		ActiveButton |= b;      /* set the proper bit   */
	} else {
		ActiveButton &= ~b;     /* clear the proper bit */
	}
}
#endif
