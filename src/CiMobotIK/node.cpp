#include <cmath>
#include "node.h"
#ifdef ENABLE_GRAPHICS
	#include <GL/gl.h>
	#include <GL/glut.h>
#endif

Node::Node(const VectorR3 &s_init, const VectorR3 &w_init, int purpose, double minTheta, double maxTheta, double initAngle) {
	this->freezed = false;
	this->m_purpose = purpose;
	this->m_seq_num_joint = -1;
	this->m_seq_num_effector = -1;
	this->m_theta = 0.0;
	this->m_theta_init = initAngle;
	this->m_theta_min = minTheta;
	this->m_theta_max = maxTheta;

	this->m_r.Set(0.0, 0.0, 0.0);		// relative global position when joint is zero
	this->m_s.Set(0.0, 0.0, 0.0);	// global position
	this->m_s_init = s_init;		// global position when joint is zero
	this->m_w.Set(0.0, 0.0, 0.0);	// global rotation axis
	this->m_w_init = w_init;		// global rotation axis when joint is zero

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
	return this->freezed;
}

/*void Node::Freeze(void) {
	this->freezed = true;
}
void Node::UnFreeze(void) {
	this->freezed = false;
}*/

#ifdef ENABLE_GRAPHICS
void Node::DrawNode(bool isRoot) {
	if ( !isRoot ) {
		glPushMatrix();

		if ( r.z != 0.0 || r.x != 0.0 ) {
			double alpha = atan2(r.z, r.x);
			glRotatef(alpha*180/M_PI, 0.0f, -1.0f, 0.0f);
		}

		if ( r.y != 0.0 ) {
			double beta = atan2(r.y, sqrt(r.x*r.x+r.z*r.z));
			glRotatef( beta*180/M_PI, 0.0f, 0.0f, 1.0f );
		}

		double length = r.Norm();
		glScalef(length/0.08, 1.0f, 1.0f);
		glTranslatef(0.04, 0.0f, 0.0f);
		glutSolidCube(0.08);
		glPopMatrix();
	}
	glTranslatef(r.x, r.y, r.z);
	glRotatef(theta*180/M_PI, v.x, v.y, v.z);
}
#endif