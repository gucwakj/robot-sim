#include "mobotik.h"

CMobotIK::CMobotIK(int num_bot, int num_targets) {
	this->m_num_bot = num_bot;
    this->m_num_targets = num_targets;
    this->m_t = 0.0;
    this->m_t_step = 0.004;
    this->m_t_count = 0;
	this->m_del_theta = new bool[NUM_DOF*num_bot + num_targets];
	this->node = new Node * [NUM_DOF*num_bot];
	this->node_right = new Node * [NUM_DOF*num_bot];
	this->node_effector = new Node * [num_targets];
	this->target_pos = new VectorR3[num_targets];
    this->target_rot = new MatrixR33[num_targets];
    this->m_j_mode = JACOB_SDLS;
    this->m_j_type = J_END;
    this->m_j_dls = TRADITIONAL;
    this->m_j_lambda = 1.1;
    this->m_reply = IK_ERROR_TIME;
}

CMobotIK::~CMobotIK(void) {
    delete this->jacob;
    delete [] this->target_pos;
    delete [] this->target_rot;
    delete [] this->m_del_theta;
	for ( int i = NUM_DOF*this->m_num_bot - 1; i >= 0; i-- ) {
		delete this->node[i];
        //delete this->node_right[i];
    }
    for ( int i = this->m_num_targets - 1; i>0; i-- ) {
        delete this->node_effector[i-1];
    }
	delete [] this->node;
    //delete [] this->node_right;
    delete [] this->node_effector;
}

void CMobotIK::iMobotAnchor(int end, double x, double y, double z, double psi, double theta, double phi, double r_le, double r_lb, double r_rb, double r_re) {
    MatrixR33 R = MatrixR33(D2R(psi), D2R(theta), D2R(phi));

	if ( end == ANCHOR_LE ) {
		double le = END_DEPTH;
		double lb = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double rb = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double re = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
        this->node[0] = new Node(R*VectorR3(le, 0, 0) + VectorR3(x, y, z), R*VectorR3( 1, 0, 0), R, JOINT, D2R(r_le));
        this->node[1] = new Node(R*VectorR3(lb, 0, 0) + VectorR3(x, y, z), R*VectorR3( 0, 0, 1), R, JOINT, D2R(r_lb), D2R(-90), D2R(90));
        this->node[2] = new Node(R*VectorR3(rb, 0, 0) + VectorR3(x, y, z), R*VectorR3( 0, 0, 1), R, JOINT, D2R(r_rb), D2R(-90), D2R(90));
        this->node[3] = new Node(R*VectorR3(re, 0, 0) + VectorR3(x, y, z), R*VectorR3( 1, 0, 0), R, JOINT, D2R(r_re));
		this->tree.insertRoot(node[0]);
		this->tree.insertLeftChild(node[0], node[1]);
		this->tree.insertLeftChild(node[1], node[2]);
		this->tree.insertLeftChild(node[2], node[3]);
	}
	else if ( end == ANCHOR_RE ) {
        double le = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
        double lb = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
        double rb = END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double re = END_DEPTH;
        this->node[0] = new Node(R*VectorR3(le, 0, 0) + VectorR3(x, y, z), R*VectorR3(-1, 0, 0), R, JOINT, D2R(r_le));
        this->node[1] = new Node(R*VectorR3(lb, 0, 0) + VectorR3(x, y, z), R*VectorR3( 0, 0,-1), R, JOINT, D2R(r_lb), D2R(-90), D2R(90));
        this->node[2] = new Node(R*VectorR3(rb, 0, 0) + VectorR3(x, y, z), R*VectorR3( 0, 0,-1), R, JOINT, D2R(r_rb), D2R(-90), D2R(90));
        this->node[3] = new Node(R*VectorR3(re, 0, 0) + VectorR3(x, y, z), R*VectorR3(-1, 0, 0), R, JOINT, D2R(r_re));
		this->tree.insertRoot(node[3]);
		this->tree.insertLeftChild(node[3], node[2]);
		this->tree.insertLeftChild(node[2], node[1]);
		this->tree.insertLeftChild(node[1], node[0]);
	}
}

void CMobotIK::iMobotAttach(int bot_num, int att_num, int face1, int face2, double r_le, double r_lb, double r_rb, double r_re) {
    VectorR3 S;
    MatrixR33 R;
    if ( face1 == 1 ) {
        S = this->node[att_num*NUM_DOF + 0]->getSInit();
        R = this->node[att_num*NUM_DOF + 0]->getRInit();
    }
    else if ( face1 == 2 || face1 == 3 ) {
        S = this->node[att_num*NUM_DOF + 1]->getSInit();
        R = this->node[att_num*NUM_DOF + 1]->getRInit();
    }
    else if ( face1 == 4 || face1 == 5 ) {
        S = this->node[att_num*NUM_DOF + 2]->getSInit();
        R = this->node[att_num*NUM_DOF + 2]->getRInit();
    }
    else if ( face1 == 6 ) {
        S = this->node[att_num*NUM_DOF + 3]->getSInit();
        R = this->node[att_num*NUM_DOF + 3]->getRInit();
    }

	/*if ( face1 == 4 && face2 == 1 ) {
		double le = BODY_WIDTH/2 + END_DEPTH;
		double lb = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double rb = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double re = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
		this->node_right[att_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit(), this->node[att_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[att_num*NUM_DOF + 2]->getThetaMin(), this->node[att_num*NUM_DOF + 2]->getThetaMax(), this->node[att_num*NUM_DOF + 2]->getThetaInit());
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*le, R[5]*le, R[8]*le), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*lb, R[5]*lb, R[8]*lb), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*rb, R[5]*rb, R[8]*rb), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*re, R[5]*re, R[8]*re), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertRightSibling(this->node[att_num*NUM_DOF + 2], this->node_right[att_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node_right[att_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 0]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 0], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
	}
	else if ( face1 == 4 && face2 == 2 ) {
		double le[3] = {BODY_LENGTH + BODY_END_DEPTH, 0, BODY_WIDTH/2};
		double lb[3] = {0, 0, BODY_WIDTH/2};
		double rb[3] = {-CENTER_LENGTH, 0, BODY_WIDTH/2};
		double re[3] = {-CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH, 0, BODY_WIDTH/2};
		this->node_right[att_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit(), this->node[att_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[att_num*NUM_DOF + 2]->getThetaMin(), this->node[att_num*NUM_DOF + 2]->getThetaMax(), this->node[att_num*NUM_DOF + 2]->getThetaInit());
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*le[0] + R[2]*le[2], R[3]*le[0] + R[5]*le[2], R[6]*le[0] + R[8]*le[2]), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*lb[0] + R[2]*lb[2], R[3]*lb[0] + R[5]*lb[2], R[6]*lb[0] + R[8]*lb[2]), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*re[0] + R[2]*re[2], R[3]*re[0] + R[5]*re[2], R[6]*re[0] + R[8]*re[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->node_right[bot_num*NUM_DOF + 0] = new Node(this->node[bot_num*NUM_DOF + 1]->getSInit(), this->node[bot_num*NUM_DOF + 1]->getWInit(), JOINT, this->node[bot_num*NUM_DOF + 1]->getThetaMin(), this->node[bot_num*NUM_DOF + 1]->getThetaMax(), this->node[bot_num*NUM_DOF + 1]->getThetaInit());

		this->tree.insertRightSibling(this->node[att_num*NUM_DOF + 2], this->node_right[att_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node_right[att_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->node[bot_num*NUM_DOF + 1]->setFrozen(1);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);

		this->tree.insertRightSibling(this->node[bot_num*NUM_DOF + 1], this->node_right[bot_num*NUM_DOF + 0]);
		this->tree.insertLeftChild(this->node_right[bot_num*NUM_DOF + 0], this->node[bot_num*NUM_DOF + 0]);
	}
	else if ( face1 == 4 && face2 == 3 ) {
		double le[3] = {-BODY_LENGTH - BODY_END_DEPTH, 0, BODY_WIDTH/2};
		double lb[3] = {0, 0, BODY_WIDTH/2};
		double rb[3] = {CENTER_LENGTH, 0, BODY_WIDTH/2};
		double re[3] = {CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH, 0, BODY_WIDTH/2};
		this->node_right[att_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit(), this->node[att_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[att_num*NUM_DOF + 2]->getThetaMin(), this->node[att_num*NUM_DOF + 2]->getThetaMax(), this->node[att_num*NUM_DOF + 2]->getThetaInit());
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*le[0] + R[2]*le[2], R[3]*le[0] + R[5]*le[2], R[6]*le[0] + R[8]*le[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*lb[0] + R[2]*lb[2], R[3]*lb[0] + R[5]*lb[2], R[6]*lb[0] + R[8]*lb[2]), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*re[0] + R[2]*re[2], R[3]*re[0] + R[5]*re[2], R[6]*re[0] + R[8]*re[2]), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->node_right[bot_num*NUM_DOF + 1] = new Node(this->node[bot_num*NUM_DOF + 1]->getSInit(), this->node[bot_num*NUM_DOF + 1]->getWInit(), JOINT, this->node[bot_num*NUM_DOF + 1]->getThetaMin(), this->node[bot_num*NUM_DOF + 1]->getThetaMax(), this->node[bot_num*NUM_DOF + 1]->getThetaInit());

		this->tree.insertRightSibling(this->node[att_num*NUM_DOF + 2], this->node_right[att_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node_right[att_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->node[bot_num*NUM_DOF + 1]->setFrozen(1);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);

		this->tree.insertRightSibling(this->node[bot_num*NUM_DOF + 1], this->node_right[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node_right[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 0]);
	}
	else if ( face1 == 4 && face2 == 4 ) {
		double le[3] = {CENTER_LENGTH + BODY_LENGTH + BODY_END_DEPTH, 0, BODY_WIDTH/2};
		double lb[3] = {CENTER_LENGTH, 0, BODY_WIDTH/2};
		double rb[3] = {0, 0, BODY_WIDTH/2};
		double re[3] = {-BODY_LENGTH - BODY_END_DEPTH, 0, BODY_WIDTH/2};
		this->node_right[att_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit(), this->node[att_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[att_num*NUM_DOF + 2]->getThetaMin(), this->node[att_num*NUM_DOF + 2]->getThetaMax(), this->node[att_num*NUM_DOF + 2]->getThetaInit());
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*le[0] + R[2]*le[2], R[3]*le[0] + R[5]*le[2], R[6]*le[0] + R[8]*le[2]), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*lb[0] + R[2]*lb[2], R[3]*lb[0] + R[5]*lb[2], R[6]*lb[0] + R[8]*lb[2]), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*re[0] + R[2]*re[2], R[3]*re[0] + R[5]*re[2], R[6]*re[0] + R[8]*re[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->node_right[bot_num*NUM_DOF + 2] = new Node(this->node[bot_num*NUM_DOF + 2]->getSInit(), this->node[bot_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[bot_num*NUM_DOF + 2]->getThetaMin(), this->node[bot_num*NUM_DOF + 2]->getThetaMax(), this->node[bot_num*NUM_DOF + 2]->getThetaInit());

		this->tree.insertRightSibling(this->node[att_num*NUM_DOF + 2], this->node_right[att_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node_right[att_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 2]);
		this->node[bot_num*NUM_DOF + 2]->setFrozen(1);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 0]);
		this->tree.insertRightSibling(this->node[bot_num*NUM_DOF + 2], this->node_right[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node_right[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
	}
	else if ( face1 == 4 && face2 == 5 ) {
		double le[3] = {-CENTER_LENGTH - BODY_LENGTH - BODY_END_DEPTH, 0, BODY_WIDTH/2};
		double lb[3] = {-CENTER_LENGTH, 0, BODY_WIDTH/2};
		double rb[3] = {0, 0, BODY_WIDTH/2};
		double re[3] = {BODY_LENGTH + BODY_END_DEPTH, 0, BODY_WIDTH/2};
		this->node_right[att_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit(), this->node[att_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[att_num*NUM_DOF + 2]->getThetaMin(), this->node[att_num*NUM_DOF + 2]->getThetaMax(), this->node[att_num*NUM_DOF + 2]->getThetaInit());
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*le[0] + R[2]*le[2], R[3]*le[0] + R[5]*le[2], R[6]*le[0] + R[8]*le[2]), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*lb[0] + R[2]*lb[2], R[3]*lb[0] + R[5]*lb[2], R[6]*lb[0] + R[8]*lb[2]), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[0]*re[0] + R[2]*re[2], R[3]*re[0] + R[5]*re[2], R[6]*re[0] + R[8]*re[2]), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->node_right[bot_num*NUM_DOF + 3] = new Node(this->node[bot_num*NUM_DOF + 2]->getSInit(), this->node[bot_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[bot_num*NUM_DOF + 2]->getThetaMin(), this->node[bot_num*NUM_DOF + 2]->getThetaMax(), this->node[bot_num*NUM_DOF + 2]->getThetaInit());
		this->tree.insertRightSibling(this->node[att_num*NUM_DOF + 2], this->node_right[att_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node_right[att_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 2]);
		this->node[bot_num*NUM_DOF + 2]->setFrozen(1);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 0]);
		this->tree.insertRightSibling(this->node[bot_num*NUM_DOF + 2], this->node_right[bot_num*NUM_DOF + 3]);
		this->tree.insertLeftChild(this->node_right[bot_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 3]);
	}
	else if ( face1 == 4 && face2 == 6 ) {
		double le = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
		double lb = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double rb = BODY_WIDTH/2 + END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double re = BODY_WIDTH/2 + END_DEPTH;
		this->node_right[att_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit(), this->node[att_num*NUM_DOF + 2]->getWInit(), JOINT, this->node[att_num*NUM_DOF + 2]->getThetaMin(), this->node[att_num*NUM_DOF + 2]->getThetaMax(), this->node[att_num*NUM_DOF + 2]->getThetaInit());
		this->node[bot_num*NUM_DOF + 0] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*le, R[5]*le, R[8]*le), VectorR3(R[2], R[5], R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_le));
		this->node[bot_num*NUM_DOF + 1] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*lb, R[5]*lb, R[8]*lb), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_lb));
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*rb, R[5]*rb, R[8]*rb), VectorR3(-R[0], -R[3], -R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
		this->node[bot_num*NUM_DOF + 3] = new Node(this->node[att_num*NUM_DOF + 2]->getSInit() + VectorR3(R[2]*re, R[5]*re, R[8]*re), VectorR3(-R[2], -R[5], -R[8]), JOINT, D2R(-180.), D2R(180.), D2R(r_re));
		this->tree.insertRightSibling(this->node[att_num*NUM_DOF + 2], this->node_right[att_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node_right[att_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 0]);
	}
	else */if ( face1 == 6 && face2 == 1 ) {
		double le = 2*END_DEPTH;
		double lb = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double rb = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double re = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
        this->node[bot_num*NUM_DOF + 0] = new Node(S + R*VectorR3(le, 0, 0), R*VectorR3( 1, 0, 0), R, JOINT, D2R(r_le));
        this->node[bot_num*NUM_DOF + 1] = new Node(S + R*VectorR3(lb, 0, 0), R*VectorR3( 0, 0, 1), R, JOINT, D2R(r_lb), D2R(-90), D2R(90));
        this->node[bot_num*NUM_DOF + 2] = new Node(S + R*VectorR3(rb, 0, 0), R*VectorR3( 0, 0, 1), R, JOINT, D2R(r_rb), D2R(-90), D2R(90));
        this->node[bot_num*NUM_DOF + 3] = new Node(S + R*VectorR3(re, 0, 0), R*VectorR3( 1, 0, 0), R, JOINT, D2R(r_re));
        this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 0]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 0], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 3]);
    }/*
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
		this->node[bot_num*NUM_DOF + 2] = new Node(this->node[att_num*NUM_DOF + 3]->getSInit() + VectorR3(R[0]*rb[0] + R[2]*rb[2], R[3]*rb[0] + R[5]*rb[2], R[6]*rb[0] + R[8]*rb[2]), VectorR3(R[0], R[3], R[6]), JOINT, D2R(-180.), D2R(180.), D2R(r_rb));
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
	}*/
	else if ( face1 == 6 && face2 == 6 ) {
		double le = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH + BODY_END_DEPTH + BODY_LENGTH;
		double lb = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH + CENTER_LENGTH;
		double rb = 2*END_DEPTH + BODY_END_DEPTH + BODY_LENGTH;
		double re = 2*END_DEPTH;
		this->node[bot_num*NUM_DOF + 0] = new Node(S + R*VectorR3(le, 0, 0), R*VectorR3(-1, 0, 0), R, JOINT, D2R(r_le));
        this->node[bot_num*NUM_DOF + 1] = new Node(S + R*VectorR3(lb, 0, 0), R*VectorR3( 0, 0, 1), R, JOINT, D2R(r_lb), D2R(-90), D2R(90));
        this->node[bot_num*NUM_DOF + 2] = new Node(S + R*VectorR3(rb, 0, 0), R*VectorR3( 0, 0, 1), R, JOINT, D2R(r_rb), D2R(-90), D2R(90));
        this->node[bot_num*NUM_DOF + 3] = new Node(S + R*VectorR3(re, 0, 0), R*VectorR3( 1, 0, 0), R, JOINT, D2R(r_re));
        this->tree.insertLeftChild(this->node[att_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 3]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 3], this->node[bot_num*NUM_DOF + 2]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 2], this->node[bot_num*NUM_DOF + 1]);
		this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + 1], this->node[bot_num*NUM_DOF + 0]);
	}
}

void CMobotIK::addEffector(int eff_num, int bot_num, int face) {
    VectorR3 eff = this->node[bot_num*NUM_DOF + face/2]->getRInit() * (4*END_DEPTH*this->node[bot_num*NUM_DOF + face/2]->getWInit());

    this->node_effector[eff_num] = new Node(this->node[bot_num*NUM_DOF + face/2]->getSInit() + eff,
                                            //VectorR3(0.0, 0.0, 0.0),
                                            this->node[bot_num*NUM_DOF + face/2]->getWInit(),
                                            this->node[bot_num*NUM_DOF + face/2]->getRInit(),
                                            EFFECTOR);
	this->tree.insertLeftChild(this->node[bot_num*NUM_DOF + face/2], this->node_effector[eff_num]);
}

void CMobotIK::setCurrentMode(int mode) {
    this->m_j_mode = mode;
}

void CMobotIK::setCurrentType(int type) {
    this->m_j_type = type;
}

void CMobotIK::setCurrentDLSMode(int mode) {
    this->m_j_dls = mode;
}

void CMobotIK::setDampingDLS(double lambda) {
    this->m_j_lambda = lambda;
}

void CMobotIK::setTarget(int num, double x, double y, double z, double psi, double theta, double phi) {
	this->target_pos[num].set(x, y, z);
    this->target_rot[num].set(D2R(psi), D2R(theta), D2R(phi));
}

void CMobotIK::setTargetPosition(int num, double x, double y, double z) {
	this->target_pos[num].set(x, y, z);
}

void CMobotIK::setTargetRotation(int num, double psi, double theta, double phi) {
	this->target_rot[num].set(D2R(psi), D2R(theta), D2R(phi));
}

int CMobotIK::getCurrentMode(void) {
	return this->jacob->getCurrentMode();
}

int CMobotIK::getCurrentType(void) {
	return this->jacob->getCurrentType();
}

int CMobotIK::getCurrentDLSMode(void) {
	return this->jacob->getCurrentDLSMode();
}

void CMobotIK::getEffectorPosition(int num, double &x, double &y, double &z) {
    VectorR3 n = this->node_effector[num]->getS();
	x = n.x;
	y = n.y;
	z = n.z;
}

double CMobotIK::getEffectorX(int num) {
    VectorR3 n = this->node_effector[num]->getS();
	return n.x;
}

double CMobotIK::getEffectorY(int num) {
    VectorR3 n = this->node_effector[num]->getS();
	return n.y;
}

double CMobotIK::getEffectorZ(int num) {
    VectorR3 n = this->node_effector[num]->getS();
	return n.z;
}

void CMobotIK::getEffectorRotation(int num, double &psi, double &theta, double &phi) {
    MatrixR33 r = this->node_effector[num]->getR();
    psi = r.psi;
    theta = r.theta;
    psi = r.phi;
}

double CMobotIK::getEffectorPsi(int num) {
    MatrixR33 r = this->node_effector[num]->getR();
    return r.psi;
}

double CMobotIK::getEffectorTheta(int num) {
    MatrixR33 r = this->node_effector[num]->getR();
    return r.theta;
}

double CMobotIK::getEffectorPhi(int num) {
    MatrixR33 r = this->node_effector[num]->getR();
    return r.phi;
}

void CMobotIK::getTargetPosition(int num, double &x, double &y, double &z) {
    x = this->target_pos[num].x;
    y = this->target_pos[num].y;
    z = this->target_pos[num].z;
}

double CMobotIK::getTargetX(int num) {
	return this->target_pos[num].x;
}

double CMobotIK::getTargetY(int num) {
	return this->target_pos[num].y;
}

double CMobotIK::getTargetZ(int num) {
	return this->target_pos[num].z;
}

void CMobotIK::getTargetRotation(int num, double &psi, double &theta, double &phi) {
	psi = this->target_rot[num].psi;
	theta = this->target_rot[num].theta;
	phi = this->target_rot[num].phi;
}

double CMobotIK::getTargetPsi(int num) {
	return this->target_rot[num].psi;
}

double CMobotIK::getTargetTheta(int num) {
	return this->target_rot[num].theta;
}

double CMobotIK::getTargetPhi(int num) {
	return this->target_rot[num].phi;
}

int CMobotIK::getNumAngles(void) {
    return NUM_DOF*this->m_num_bot;
}

void CMobotIK::getAngles(double *array) {
    for ( int i = 0; i < this->getNumAngles(); i++ ) {
        array[i] = R2D(this->node[i]->getTheta());
        if ( fabs(array[i]) < 0.5 ) {
            array[i] = 0;
        }
    }
}

int CMobotIK::getReplyMessage(void) {
    return this->m_reply;
}

void CMobotIK::formatAngles(int method, double *array) {
    double *angles = new double[this->getNumAngles()];
    this->getAngles(angles);

    if ( method == 1 ) {
        for ( int i = 0; i < 8; i++ ) {
            array[i*8 + 7 - i] = angles[7-i];
            if ( (i%NUM_DOF == 1) || (i%NUM_DOF == 2) ) {
                for ( int j = i; j < 8; j++ ) {
                    array[i*8 + 7 - i + 8*(7-j)] = angles[7-i];
                }
            }
        }
    }
    else if ( method == 2 ) {
        for ( int i = 0; i < 8; i++ ) {
            array[i*8 + i] = angles[i];
            if ( (i%NUM_DOF == 1) || (i%NUM_DOF == 2) ) {
                for ( int j = i; j < 8; j++ ) {
                    array[i*8 + i + 8*(7-j)] = angles[i];
                }
            }
        }
    }
    delete angles;
}

void CMobotIK::computeInverseKinematics(void) {
	bool loop = true;
    this->jacob = new Jacobian(&(this->tree), this->target_pos, this->target_rot);
    this->jacob->setCurrentMode(this->m_j_mode);
    this->jacob->setCurrentType(this->m_j_type);
    this->jacob->setCurrentDLSMode(this->m_j_dls);
    this->jacob->setDampingDLS(this->m_j_lambda);
	this->tree.init();
	this->tree.compute();
	this->print_intermediate_data();

	while( loop ) {
		this->jacob->computeJacobian();			// set up Jacobian and deltaS vectors
		this->jacob->calcDeltaThetas();			// calculate delta Theta values
		this->jacob->updateThetas();			// apply the change in the theta values
		this->jacob->updateClampMag();		    // update distance to target position

		this->print_intermediate_data();        // print data for analysis purposes

		this->update_targets();					// update target to new values
		this->set_flags();						// set flags for completion
		this->increment_step();					// increment time step
		loop = this->end_simulation();			// check to end simulation
    }
    this->print_intermediate_data();            // print last step of data
}

void CMobotIK::print_intermediate_data(void) {
	cout << this->m_t_count << "\t" << this->m_t << "\t";
	cout << this->m_t_count << "\t";

    /*for ( int i = 0; i < this->getNumAngles(); i++ ) {
		if ( this->node[i] ) {
			cout << setw(12) << R2D(this->node[i]->getTheta());
		}
	}
	cout << "\t";
    cout << endl;*/

	/*for ( int i = 0; i < this->m_num_targets; i++ ) {
		if ( this->node_effector[i] ) {
			cout << this->node_effector[i]->getS() << this->target_pos[i];
			//MatrixR33 R = this->node_effector[i]->getR();
            //cout << "Rot: " << R.getEulerAngles() << "\t" << this->target_rot[i].getEulerAngles() << endl;
		}
	}
	cout << endl;*/

	/*for ( int i = 0; i < this->m_num_bot*NUM_DOF+this->m_num_targets; i++ ) {
			if ( this->node[i] ) {
				cout << "Node " << i << endl;
				cout << "     S: " << this->node[i]->getS() << endl;
				cout << "S Init: " << this->node[i]->getSInit() << endl;
				cout << "     W: " << this->node[i]->getW() << endl;
                cout << "W Init: " << this->node[i]->getWInit() << endl;
                MatrixR33 R = this->node[i]->getR();
                MatrixR33 R1 = this->node[i]->getRInit();
                cout << "  R EE: " << R.getEulerAngles() << endl;
                cout << "R Init: " << R1.getEulerAngles() << endl;
                cout << "     R: " << R.m11 << " " << R.m12 << " " << R.m13 << endl;
                cout << "     R: " << R.m21 << " " << R.m22 << " " << R.m23 << endl;
                cout << "     R: " << R.m31 << " " << R.m32 << " " << R.m33 << endl;
				cout << " Theta: " << this->node[i]->getTheta() << endl;
			}
	}
	cout << endl;*/
	cout << endl;
}

void CMobotIK::update_targets(void) {
	//this->target[0].Set(3.f, 0.1f, 0.3f);
}

void CMobotIK::increment_step(void) {
	this->m_t += this->m_t_step;
	this->m_t_count++;
}

bool CMobotIK::end_simulation(void) {
	if ( this->m_t_count == 2000 ) {
        this->m_reply = IK_ERROR_TIME;
        return false;
    }
	if ( is_true(this->m_del_theta, NUM_DOF*this->m_num_bot) ) {
        this->m_reply = IK_SUCCESS;
        return false;
    }
	return true;
}

void CMobotIK::set_flags(void) {
	Node *n = this->tree.getRoot();
	int i = 0;
	while ( n ) {
        this->m_del_theta[i] = 1;
        if ( n->isJoint() && (fabs(this->jacob->getDeltaTheta(n->getJointNum())) > 0.005) )
			this->m_del_theta[i] = 0;
		i++;
		n = this->tree.getSuccessor(n);
	}
}

bool CMobotIK::is_true(bool *a, int length) {
	for ( int i = 0; i < length; i++ ) {
		if ( a[i] == false )
			return false;
	}
	return true;
}

inline double CMobotIK::D2R(double deg) {
	return deg*M_PI/180;
}

inline double CMobotIK::R2D(double rad) {
	return rad/M_PI*180;
}