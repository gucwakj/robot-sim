#include <float.h>
#include "matrixR33.h"

MatrixR33::MatrixR33(void) {
    this->m11 = 0;	this->m12 = 0;	this->m13 = 0;
    this->m21 = 0;	this->m22 = 0;	this->m23 = 0;
    this->m31 = 0;	this->m32 = 0;	this->m33 = 0;
    this->psi = 0;  this->theta = 0;this->phi = 0;
}

MatrixR33::MatrixR33(double psi, double theta, double phi) {
    double  spsi = sin(psi),    cpsi = cos(psi),
            stheta = sin(theta),ctheta = cos(theta),
            sphi = sin(phi),    cphi = cos(phi);

    this->m11 =  cphi*ctheta;
    this->m12 = -cphi*stheta*cpsi + sphi*spsi;
    this->m13 =  cphi*stheta*spsi + sphi*cpsi;
    this->m21 =  stheta;
    this->m22 =  ctheta*cpsi;
    this->m23 = -ctheta*spsi;
    this->m31 = -sphi*ctheta;
    this->m32 =  sphi*stheta*cpsi + cphi*spsi;
    this->m33 = -sphi*stheta*spsi + cphi*cpsi;
    this->psi = psi;
    this->theta = theta;
    this->phi = phi;
}

MatrixR33::MatrixR33(double a11, double a21, double a31, double a12, double a22, double a32, double a13, double a23, double a33) {
    this->m11 = a11;	this->m12 = a12;	this->m13 = a13;
    this->m21 = a21;	this->m22 = a22;	this->m23 = a23;
    this->m31 = a31;	this->m32 = a32;	this->m33 = a33;
}

void MatrixR33::set(double psi, double theta, double phi) {
    double  spsi = sin(psi),    cpsi = cos(psi),
            stheta = sin(theta),ctheta = cos(theta),
            sphi = sin(phi),    cphi = cos(phi);

    this->m11 =  cphi*ctheta;
    this->m12 = -cphi*stheta*cpsi + sphi*spsi;
    this->m13 =  cphi*stheta*spsi + sphi*cpsi;
    this->m21 =  stheta;
    this->m22 =  ctheta*cpsi;
    this->m23 = -ctheta*spsi;
    this->m31 = -sphi*ctheta;
    this->m32 =  sphi*stheta*cpsi + cphi*spsi;
    this->m33 = -sphi*stheta*spsi + cphi*cpsi;
    this->psi = psi;
    this->theta = theta;
    this->phi = phi;
}

void MatrixR33::set(double a11, double a21, double a31, double a12, double a22, double a32, double a13, double a23, double a33) {
    this->m11 = a11;    this->m12 = a12;    this->m13 = a13;
    this->m21 = a21;    this->m22 = a22;    this->m23 = a23;
    this->m31 = a31;    this->m32 = a32;    this->m33 = a33;
}

void MatrixR33::setColumn1(double a11, double a21, double a31) {
	this->m11 = a11;    this->m21 = a21;    this->m31 = a31;
}

void MatrixR33::setColumn2(double a12, double a22, double a32) {
    this-> m12 = a12;   this->m22 = a22;    this->m32 = a32;
}

void MatrixR33::setColumn3(double a13, double a23, double a33) {
    this->m13 = a13;    this->m23 = a23;    this->m33 = a33;
}

void MatrixR33::setIdentity(void) {
    this->m11 = this->m22 = this->m33 = 1.0;
    this->m12 = this->m13 = this->m21 = this->m23 = this->m31 = this->m32 = 0.0;
}

void MatrixR33::setZero(void) {
    this->m11 = this->m12 = this->m13 = 0,
    this->m21 = this->m22 = this->m23 = 0,
    this->m31 = this->m32 = this->m33 = 0;
}

VectorR3 MatrixR33::getColumn1(void) {
	return VectorR3(this->m11, this->m21, this->m31);
}

VectorR3 MatrixR33::getColumn2(void) {
	return VectorR3(this->m12, this->m22, this->m32);
}

VectorR3 MatrixR33::getColumn3(void) {
	return VectorR3(this->m13, this->m23, this->m33);
}

VectorR3 MatrixR33::getEulerAngles(void) {
    return VectorR3(this->psi, this->theta, this->phi);
}

void MatrixR33::transpose(void) {
    double temp = this->m12;
    this->m12 = this->m21;
    this->m21 = temp;

    temp = this->m13;
    this->m13 = this->m31;
    this->m31 = temp;

    temp = this->m23;
    this->m23 = this->m32;
    this->m32 = temp;
}

void MatrixR33::transform(VectorR3 *u) {
	double newX = m11*u->x + m12*u->y + m13*u->z;
	double newY = m21*u->x + m22*u->y + m23*u->z;
	u->z = m31*u->x + m32*u->y + m33*u->z;
	u->x = newX;
	u->y = newY;
}

void MatrixR33::transform(const VectorR3& src, VectorR3 *dest) {
	dest->x = m11*src.x + m12*src.y + m13*src.z;
	dest->y = m21*src.x + m22*src.y + m23*src.z;
	dest->z = m31*src.x + m32*src.y + m33*src.z;
}

// Re-normalizes nearly orthonormal matrix
MatrixR33& MatrixR33::reNormalize()	{
	double alpha = m11*m11+m21*m21+m31*m31;	// First column's norm squared
	double beta  = m12*m12+m22*m22+m32*m32;	// Second column's norm squared
	double gamma = m13*m13+m23*m23+m33*m33;	// Third column's norm squared
	alpha = 1.0 - 0.5*(alpha-1.0);				// Get mult. factor
	beta  = 1.0 - 0.5*(beta-1.0);
	gamma = 1.0 - 0.5*(gamma-1.0);
	m11 *= alpha;								// Renormalize first column
	m21 *= alpha;
	m31 *= alpha;
	m12 *= beta;								// Renormalize second column
	m22 *= beta;
	m32 *= beta;
	m13 *= gamma;
	m23 *= gamma;
	m33 *= gamma;
	alpha = m11*m12+m21*m22+m31*m32;		// First and second column dot product
	beta  = m11*m13+m21*m23+m31*m33;		// First and third column dot product
	gamma = m12*m13+m22*m23+m32*m33;		// Second and third column dot product
	alpha *= 0.5;
	beta *= 0.5;
	gamma *= 0.5;
	double temp1, temp2;
	temp1 = m11-alpha*m12-beta*m13;			// Update row1
	temp2 = m12-alpha*m11-gamma*m13;
	m13 -= beta*m11+gamma*m12;
	m11 = temp1;
	m12 = temp2;
	temp1 = m21-alpha*m22-beta*m23;			// Update row2
	temp2 = m22-alpha*m21-gamma*m23;
	m23 -= beta*m21+gamma*m22;
	m21 = temp1;
	m22 = temp2;
	temp1 = m31-alpha*m32-beta*m33;			// Update row3
	temp2 = m32-alpha*m31-gamma*m33;
	m33 -= beta*m31+gamma*m32;
	m31 = temp1;
	m32 = temp2;
	return *this;
}

/*
 * rotation matrix from angle and axis based upon
 * Rodrigues Rotation Formula
 */
MatrixR33& MatrixR33::rotate(double theta, const VectorR3& w) {
    double c = cos(theta);
    double s = sin(theta);

    double r11 = c + w.x*w.x*(1-c);
    double r21 = w.x*w.y*(1-c) + w.z*s;
    double r31 = w.x*w.z*(1-c) - w.y*s;
    double r12 = w.x*w.y*(1-c) - w.z*s;
    double r22 = c + w.y*w.y*(1-c);
    double r32 = w.y*w.z*(1-c) + w.x*s;
    double r13 = w.x*w.z*(1-c) + w.y*s;
    double r23 = w.y*w.z*(1-c) - w.x*s;
    double r33 = c + w.z*w.z*(1-c);

    // [this->m]*[r]
    double temp11 = this->m11*r11 + this->m12*r21 + this->m13*r31;
    double temp21 = this->m21*r11 + this->m22*r21 + this->m23*r31;
    double temp31 = this->m31*r11 + this->m32*r21 + this->m33*r31;
    double temp12 = this->m11*r12 + this->m12*r22 + this->m13*r32;
    double temp22 = this->m21*r12 + this->m22*r22 + this->m23*r32;
    double temp32 = this->m31*r12 + this->m32*r22 + this->m33*r32;
    double temp13 = this->m11*r13 + this->m12*r23 + this->m13*r33;
    double temp23 = this->m21*r13 + this->m22*r23 + this->m23*r33;
    double temp33 = this->m31*r13 + this->m32*r23 + this->m33*r33;
    this->m11 = temp11;
    this->m21 = temp21;
    this->m31 = temp31;
    this->m12 = temp12;
    this->m22 = temp22;
    this->m32 = temp32;
    this->m13 = temp13;
    this->m23 = temp23;
    this->m33 = temp33;

    if ( fabs(this->m21 - 1) < DBL_EPSILON ) {         // m21 == 1
        this->psi = atan2(this->m32, this->m33);
        this->theta = M_PI/2;
        this->phi = 0;
    }
    else if ( fabs(this->m21 + 1) < DBL_EPSILON ) {    // m21 == -1
        this->psi = atan2(this->m13, this->m33);
        this->theta = -M_PI/2;
        this->phi = 0;
    }
    else {
        this->theta = asin(this->m21);
        this->psi = atan2(-this->m23/cos(this->theta), this->m22/cos(this->theta));
        this->phi = atan2(-this->m31/cos(this->theta), this->m11/cos(this->theta));
    }

    return ( *this );
}

void MatrixR33::recomputeEulerAngles(void) {
    if ( fabs(this->m21 - 1) < DBL_EPSILON ) {         // m21 == 1
        this->psi = atan2(this->m32, this->m33);
        this->theta = M_PI/2;
        this->phi = 0;
    }
    else if ( fabs(this->m21 + 1) < DBL_EPSILON ) {    // m21 == -1
        this->psi = atan2(this->m13, this->m33);
        this->theta = -M_PI/2;
        this->phi = 0;
    }
    else {
        this->theta = asin(this->m21);
        this->psi = atan2(-this->m23/cos(this->theta), this->m22/cos(this->theta));
        this->phi = atan2(-this->m31/cos(this->theta), this->m11/cos(this->theta));
    }
}

MatrixR33& MatrixR33::multiply(const MatrixR33& A) {
    double temp11 = A.m11*this->m11 + A.m12*this->m21 + A.m13*this->m31;
    double temp21 = A.m21*this->m11 + A.m22*this->m21 + A.m23*this->m31;
    double temp31 = A.m31*this->m11 + A.m32*this->m21 + A.m33*this->m31;
    double temp12 = A.m11*this->m12 + A.m12*this->m22 + A.m13*this->m32;
    double temp22 = A.m21*this->m12 + A.m22*this->m22 + A.m23*this->m32;
    double temp32 = A.m31*this->m12 + A.m32*this->m22 + A.m33*this->m32;
    double temp13 = A.m11*this->m13 + A.m12*this->m23 + A.m13*this->m33;
    double temp23 = A.m21*this->m13 + A.m22*this->m23 + A.m23*this->m33;
    double temp33 = A.m31*this->m13 + A.m32*this->m23 + A.m33*this->m33;
    this->m11 = temp11;
    this->m21 = temp21;
    this->m31 = temp31;
    this->m12 = temp12;
    this->m22 = temp22;
    this->m32 = temp32;
    this->m13 = temp13;
    this->m23 = temp23;
    this->m33 = temp33;

    this->recomputeEulerAngles();

    return (*this);
}

MatrixR33& MatrixR33::operator*= (const MatrixR33& A) {
    double temp11 = A.m11*this->m11 + A.m12*this->m21 + A.m13*this->m31;
    double temp21 = A.m21*this->m11 + A.m22*this->m21 + A.m23*this->m31;
    double temp31 = A.m31*this->m11 + A.m32*this->m21 + A.m33*this->m31;
    double temp12 = A.m11*this->m12 + A.m12*this->m22 + A.m13*this->m32;
    double temp22 = A.m21*this->m12 + A.m22*this->m22 + A.m23*this->m32;
    double temp32 = A.m31*this->m12 + A.m32*this->m22 + A.m33*this->m32;
    double temp13 = A.m11*this->m13 + A.m12*this->m23 + A.m13*this->m33;
    double temp23 = A.m21*this->m13 + A.m22*this->m23 + A.m23*this->m33;
    double temp33 = A.m31*this->m13 + A.m32*this->m23 + A.m33*this->m33;
    this->m11 = temp11;
    this->m21 = temp21;
    this->m31 = temp31;
    this->m12 = temp12;
    this->m22 = temp22;
    this->m32 = temp32;
    this->m13 = temp13;
    this->m23 = temp23;
    this->m33 = temp33;

    this->recomputeEulerAngles();

    return (*this);
}

ostream& operator<< ( ostream& os, const MatrixR33& A ) {
	os << " <" << A.m11 << ", " << A.m12 << ", " << A.m13  << ">\n"
	<< " <" << A.m21 << ", " << A.m22 << ", " << A.m23  << ">\n"
	<< " <" << A.m31 << ", " << A.m32 << ", " << A.m33  << ">\n" ;
	return (os);
}
