#include <cmath>
#include "node.h"

Node::Node(const VectorR3 &s_init, const VectorR3 &w_init, const MatrixR33 &R_init, int purpose, double theta_init, double theta_min, double theta_max) {
    this->m_frozen = false;
    this->m_purpose = purpose;
    this->m_seq_num_joint = -1;
    this->m_seq_num_effector = -1;
    this->m_theta = 0.0;
    this->m_theta_init = theta_init;
    this->m_theta_min = theta_min;
    this->m_theta_max = theta_max;

    this->m_R.set(0.0, 0.0, 0.0);       // global rotation matrix
    this->m_R_init = R_init;            // global rotation matrix when joint is zero
    this->m_r.set(0.0, 0.0, 0.0);       // relative global position when joint is zero
    this->m_s.set(0.0, 0.0, 0.0);       // global position
    this->m_s_init = s_init;            // global position when joint is zero
    this->m_w.set(0.0, 0.0, 0.0);       // global rotation axis
    this->m_w_init = w_init;            // global rotation axis when joint is zero

    this->left = 0;
    this->right = 0;
    this->realparent = 0;
}

void Node::setSeqNum(int seq_num) {
	switch ( this->m_purpose ) {
		case JOINT:
			this->m_seq_num_joint = seq_num;
			this->m_seq_num_effector = -1;
			break;
		case EFFECTOR:
			this->m_seq_num_effector = seq_num;
			this->m_seq_num_joint = -1;
			break;
	}
}

void Node::setFrozen(bool s) {
	this->m_frozen = s;
}

const MatrixR33& Node::getR(void) {
    return this->m_R;
}

const MatrixR33& Node::getRInit(void) {
    return this->m_R_init;
}

const VectorR3& Node::getS(void) {
	return this->m_s;
}

const VectorR3& Node::getSInit(void) {
	return this->m_s_init;
}

const VectorR3& Node::getW(void) {
	return this->m_w;
}

const VectorR3& Node::getWInit(void) {
	return this->m_w_init;
}

double Node::getTheta(void) {
	return this->m_theta;
}

double Node::getThetaInit(void) {
	return this->m_theta_init;
}

double Node::getThetaMin(void) {
	return this->m_theta_min;
}

double Node::getThetaMax(void) {
	return this->m_theta_max;
}

int Node::getEffectorNum(void) {
	return this->m_seq_num_effector;
}

int Node::getJointNum(void) {
	return this->m_seq_num_joint;
}

int Node::getPurpose(void) {
	return this->m_purpose;
}

bool Node::isEffector(void) {
    return (this->m_purpose == EFFECTOR);
}

bool Node::isJoint(void) {
    return (this->m_purpose == JOINT);
}

bool Node::isFrozen(void) {
    return this->m_frozen;
}

double Node::updateTheta(double delta) {
    this->m_theta += delta;
    return this->m_theta;
}

// Compute the global rotation matrix of a single node
void Node::computeR(void) {
    Node *y = this->realparent;
    this->m_R = this->m_R_init;         // Initialize to local rotation matrix
    while ( y ) {
        this->m_R.rotate(y->m_theta, y->m_w_init);
        y = y->realparent;
    }
}

// Compute the global position of a single node
void Node::computeS(void) {
	Node *y = this->realparent;
	Node *z = this;
	this->m_s = this->m_r;				// Initialize to local (relative) position
	while ( y ) {
		this->m_s.rotate(y->m_theta, y->m_w_init);
		y = y->realparent;
		z = z->realparent;
		this->m_s += z->m_r;
	}
}

// Compute the global rotation axis of a single node
void Node::computeW(void) {
	Node *y = this->realparent;
	this->m_w = this->m_w_init;			// Initialize to local rotation axis
	while ( y ) {
		this->m_w.rotate(y->m_theta, y->m_w_init);
		y = y->realparent;
	}
}

void Node::initNode(void) {
    this->m_theta = this->m_theta_init;
}
