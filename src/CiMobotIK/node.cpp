#include <cmath>
#include "node.h"
#ifdef ENABLE_GRAPHICS
	#include <GL/gl.h>
	#include <GL/glut.h>
#endif

Node::Node(const VectorR3 &attach, const VectorR3 &v, int purpose, double minTheta, double maxTheta, double restAngle) {
	this->freezed = false;
	this->purpose = purpose;
	this->seqNumJoint = -1;
	this->seqNumEffector = -1;
	this->attach = attach;			// Global attachment point when joints are at zero angle
	this->r.Set(0.0, 0.0, 0.0);		// r will be updated when this node is inserted into tree
	this->v = v;					// Rotation axis when joints at zero angles
	//this->theta = 0.0;
	this->theta = restAngle;
	//this->minTheta = minTheta;
	//this->maxTheta = maxTheta;
	//this->restAngle = restAngle;
	this->left = this->right = this->realparent = 0;
}

// Compute the global position of a single node
void Node::ComputeS(void) {
	Node *y = this->realparent;
	Node *w = this;
	this->s = r;							// Initialize to local (relative) position
	while ( y ) {
		this->s.Rotate( y->theta, y->v );
		y = y->realparent;
		w = w->realparent;
		this->s += w->r;
	}
}

// Compute the global rotation axis of a single node
void Node::ComputeW(void) {
	Node *y = this->realparent;
	w = v;							// Initialize to local rotation axis
	while (y) {
		w.Rotate(y->theta, y->v);
		y = y->realparent;
	}
}

void Node::PrintNode(void) {
	cout << "Attach : (" << attach << ")" << endl;
	cout << "r : (" << r << ")" << endl;
	cout << "s : (" << s << ")" << endl;
	cout << "w : (" << w << ")" << endl;
	cout << "realparent : " << realparent->seqNumJoint << endl;
}

void Node::InitNode(void) {
	//this->theta = 0.0;
}

const VectorR3& Node::GetAttach(void) {
	return this->attach;
}

double Node::GetTheta(void) {
	return this->theta;
}

double Node::AddToTheta(double delta) {
	this->theta += delta;
	return this->theta;
}

const VectorR3& Node::GetS(void) {
	return this->s;
}

const VectorR3& Node::GetW(void) {
	return this->w;
}

/*double Node::GetMinTheta(void) {
	return minTheta;
}

double Node::GetMaxTheta(void){
	return maxTheta;
}*/

/*double Node::GetRestAngle(void) {
	return restAngle;
}*/

/*void Node::SetTheta(double newTheta) {
	theta = newTheta;
}*/

bool Node::IsEffector(void) {
	return this->purpose == EFFECTOR;
}
int Node::GetEffectorNum(void) {
	return this->seqNumEffector;
}
bool Node::IsJoint(void) {
	return this->purpose == JOINT;
}
int Node::GetJointNum(void) {
	return this->seqNumJoint;
}

bool Node::IsFrozen(void) {
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