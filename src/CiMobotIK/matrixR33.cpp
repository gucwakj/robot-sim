#include "matrixR33.h"

MatrixR33::MatrixR33(void) {
	m11 = 0;	m12 = 0;	m13 = 0;
	m21 = 0;	m22 = 0;	m23 = 0;
	m31 = 0;	m32 = 0;	m33 = 0;
}

MatrixR33::MatrixR33(const VectorR3& u, const VectorR3& v, const VectorR3& w) {
	m11 = u.x;	m12 = v.x;	m13 = w.x;
	m21 = u.y;	m22 = v.y;	m23 = w.y;
	m31 = u.z;	m32 = v.z;	m33 = w.z;
}

MatrixR33::MatrixR33(double psi, double theta, double phi) {
    double R[9];
    rotation_matrix_from_euler_angles(R, psi, theta, phi);
    m11 = R[0];  m12 = R[1];  m13 = R[2];
    m21 = R[3];  m22 = R[4];  m23 = R[5];
    m31 = R[6];  m32 = R[7];  m33 = R[8];
    this->psi = psi;
    this->theta = theta;
    this->phi = phi;
}

MatrixR33::MatrixR33(double a11, double a21, double a31, double a12, double a22, double a32, double a13, double a23, double a33) {
	m11 = a11;	m12 = a12;	m13 = a13;
	m21 = a21;	m22 = a22;	m23 = a23;
	m31 = a31;	m32 = a32;	m33 = a33;
}

void MatrixR33::set( const VectorR3& u, const VectorR3& v, const VectorR3& w) {
	m11 = u.x;	m12 = v.x;	m13 = w.x;
	m21 = u.y;	m22 = v.y;	m23 = w.y;
	m31 = u.z;	m32 = v.z;	m33 = w.z;
}

void MatrixR33::set(double psi, double theta, double phi) {
    double R[9];
    rotation_matrix_from_euler_angles(R, psi, theta, phi);
    m11 = R[0];  m12 = R[1];  m13 = R[2];
    m21 = R[3];  m22 = R[4];  m23 = R[5];
    m31 = R[6];  m32 = R[7];  m33 = R[8];
    this->psi = psi;
    this->theta = theta;
    this->phi = phi;
}

void MatrixR33::set(double a11, double a21, double a31, double a12, double a22, double a32, double a13, double a23, double a33) {
	m11 = a11;	m12 = a12;	m13 = a13;
	m21 = a21;	m22 = a22;	m23 = a23;
	m31 = a31;	m32 = a32;	m33 = a33;
}

void MatrixR33::setColumn1(double a11, double a21, double a31) {
	m11 = a11; m21 = a21; m31 = a31;
}

void MatrixR33::setColumn2(double a12, double a22, double a32) {
	m12 = a12; m22 = a22; m32 = a32;
}

void MatrixR33::setColumn3(double a13, double a23, double a33) {
	m13 = a13; m23 = a23; m33 = a33;
}

void MatrixR33::setColumn1(const VectorR3& u) {
	m11 = u.x; m21 = u.y; m31 = u.z;
}

void MatrixR33::setColumn2(const VectorR3& v) {
	m12 = v.x; m22 = v.y; m32 = v.z;
}

void MatrixR33::setColumn3(const VectorR3& w) {
	m13 = w.x; m23 = w.y; m33 = w.z;
}

void MatrixR33::setDiagonal(double a11, double a22, double a33) {
	m11 = a11;	m22 = a22;	m33 = a33;
}

void MatrixR33::setDiagonal(const VectorR3& u) {
	this->setDiagonal(u.x, u.y, u.z);
}

void MatrixR33::setIdentity(void) {
	m11 = m22 = m33 = 1.0;
	m12 = m13 = m21 = m23 = m31 = m32 = 0.0;
}

void MatrixR33::setZero(void) {
	m11 = m12 = m13 = m21 = m22 = m23 = m31 = m32 = m33 = 0.0;
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

VectorR3 MatrixR33::getDiagonal(void) {
	return VectorR3(this->m11, this->m22, this->m33);
}

void MatrixR33::transpose(void) {
	double temp = m12;
	m12 = m21;
	m21 = temp;
	temp = m13;
	m13 = m31;
	m31 = temp;
	temp = m23;
	m23 = m32;
	m32 = temp;
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


void rotation_matrix_from_euler_angles(double *R, double psi, double theta, double phi) {
    double  sphi = sin(phi),    cphi = cos(phi),
    stheta = sin(theta),ctheta = cos(theta),
    spsi = sin(psi),    cpsi = cos(psi);

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

ostream& operator<< ( ostream& os, const MatrixR33& A ) {
	os << " <" << A.m11 << ", " << A.m12 << ", " << A.m13  << ">\n"
	<< " <" << A.m21 << ", " << A.m22 << ", " << A.m23  << ">\n"
	<< " <" << A.m31 << ", " << A.m32 << ", " << A.m33  << ">\n" ;
	return (os);
}
