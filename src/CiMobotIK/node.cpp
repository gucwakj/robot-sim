#include <cmath>
#include "node.h"

Node::Node(const VectorR3 &s_init, const VectorR3 &w_init, int purpose, double minTheta, double maxTheta, double initAngle) {
	this->m_frozen = false;
	this->m_purpose = purpose;
	this->m_seq_num_joint = -1;
	this->m_seq_num_effector = -1;
	this->m_theta = 0.0;
	this->m_theta_init = initAngle;
	this->m_theta_min = minTheta;
	this->m_theta_max = maxTheta;

	this->m_r.Set(0.0, 0.0, 0.0);		// relative global position when joint is zero
	this->m_s.Set(0.0, 0.0, 0.0);		// global position
	this->m_s_init = s_init;			// global position when joint is zero
	this->m_w.Set(0.0, 0.0, 0.0);		// global rotation axis
	this->m_w_init = w_init;			// global rotation axis when joint is zero
	//this->m_o.Set(
	//this->m_o_init =

	this->left = 0;
	this->right = 0;
	this->realparent = 0;
}

void Node::initNode(void) {
	this->m_theta = this->m_theta_init;
}

double Node::updateTheta(double delta) {
	this->m_theta += delta;
	return this->m_theta;
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

// Compute the global position of a single node
void Node::computeS(void) {
	Node *y = this->realparent;
	Node *z = this;
	this->m_s = this->m_r;				// Initialize to local (relative) position
	while ( y ) {
		this->m_s.Rotate( y->m_theta, y->m_w_init );
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
		this->m_w.Rotate(y->m_theta, y->m_w_init);
		y = y->realparent;
	}
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
