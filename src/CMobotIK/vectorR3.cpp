#include "vectorR3.h"

VectorR3::VectorR3(void) {
	this->x = 0;
	this->y = 0;
	this->z = 0;
}

VectorR3::VectorR3(double x, double y, double z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

void VectorR3::set(double x, double y, double z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

double VectorR3::norm(void) {
    return sqrt(x*x + y*y + z*z);
}

void VectorR3::dump(double *v) const {
    *v = x;
    *(v+1) = y;
    *(v+2) = z;
}

void VectorR3::rotate(double theta, VectorR3& w) {
	double c = cos(theta);
	double s = sin(theta);
	double dotw = (x*w.x + y*w.y + z*w.z);
	double v0x = dotw*w.x;
	double v0y = dotw*w.y;		    // v0 = provjection onto w
	double v0z = dotw*w.z;
	double v1x = x-v0x;
	double v1y = y-v0y;			    // v1 = projection onto plane normal to w
	double v1z = z-v0z;
	double v2x = w.y*v1z - w.z*v1y;
	double v2y = w.z*v1x - w.x*v1z;	// v2 = w * v1 (cross product)
	double v2z = w.x*v1y - w.y*v1x;

	this->x = v0x + c*v1x + s*v2x;
	this->y = v0y + c*v1y + s*v2y;
	this->z = v0z + c*v1z + s*v2z;
}

double VectorR3::operator[] (int i) {
	switch (i) {
		case 0:
			return x;
		case 1:
			return y;
		case 2:
			return z;
		default:
			return 0.0;
	}
}

VectorR3 VectorR3::operator- (void) {
	return (VectorR3(-this->x, -this->y, -this->z) );
}

VectorR3& VectorR3::operator= (const VectorR3& v) {
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
	return (*this);
}

VectorR3& VectorR3::operator+= (const VectorR3& v) {
	this->x += v.x;
	this->y += v.y;
	this->z += v.z;
	return (*this);
}

VectorR3& VectorR3::operator-= (const VectorR3& v) {
	this->x -= v.x;
	this->y -= v.y;
	this->z -= v.z;
	return (*this);
}

VectorR3& VectorR3::operator*= (double m) {
	this->x *= m;
	this->y *= m;
	this->z *= m;
	return (*this);
}

VectorR3& VectorR3::operator/= (double m) {
	double mInv = 1.0/m;
	this->x *= mInv;
	this->y *= mInv;
	this->z *= mInv;
	return (*this);
}

VectorR3& VectorR3::operator*= (const VectorR3& v) {
	double	tx = this->x,
			ty = this->y;
	this->x =  y*v.z -  z*v.y;
	this->y =  z*v.x - tx*v.z;
	this->z = tx*v.y - ty*v.x;
	return (*this);
}

int operator== (const VectorR3& u, const VectorR3& v) {
	return ( u.x==v.x && u.y==v.y && u.z==v.z );
}

VectorR3 operator* (double m, const VectorR3& u) {
	return VectorR3( u.x*m, u.y*m, u.z*m);
}

VectorR3 operator* (const VectorR3& u, double m) {
	return VectorR3( u.x*m, u.y*m, u.z*m);
}

VectorR3 operator/ (const VectorR3& u, double m) {
	double mInv = 1.0/m;
	return VectorR3( u.x*mInv, u.y*mInv, u.z*mInv);
}

VectorR3 operator+ (const VectorR3& u, const VectorR3& v) {
	return VectorR3(u.x+v.x, u.y+v.y, u.z+v.z);
}

VectorR3 operator- (const VectorR3& u, const VectorR3& v) {
	return VectorR3(u.x-v.x, u.y-v.y, u.z-v.z);
}

VectorR3 operator* (const VectorR3& u, const VectorR3& v) {
	return (VectorR3(	u.y*v.z - u.z*v.y,
						u.z*v.x - u.x*v.z,
						u.x*v.y - u.y*v.x) );
}

ostream& operator<< ( ostream& os, const VectorR3& u ) {
	return (os << "<" << u.x << "," << u.y << "," << u.z << ">");
}