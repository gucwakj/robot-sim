#include "linearR3.h"

VectorR3::VectorR3(void) {
	this->x = 0;
	this->y = 0;
	this->z = 0;
}

VectorR3::VectorR3(double xVal, double yVal, double zVal) {
	this->x = xVal;
	this->y = yVal;
	this->z = zVal;
}

double VectorR3::MaxAbs() {
	register double m;
	m = (x>0.0) ? x : -x;
	if ( y>m ) m=y;
	else if ( -y >m ) m = -y;
	if ( z>m ) m=z;
	else if ( -z>m ) m = -z;
	return m;
}

VectorR3& VectorR3::Rotate( double theta, const VectorR3& w) {
	double c = cos(theta);
	double s = sin(theta);
	double dotw = (x*w.x + y*w.y + z*w.z);
	double v0x = dotw*w.x;
	double v0y = dotw*w.y;		// v0 = provjection onto w
	double v0z = dotw*w.z;
	double v1x = x-v0x;
	double v1y = y-v0y;			// v1 = projection onto plane normal to w
	double v1z = z-v0z;
	double v2x = w.y*v1z - w.z*v1y;
	double v2y = w.z*v1x - w.x*v1z;	// v2 = w * v1 (cross product)
	double v2z = w.x*v1y - w.y*v1x;

	x = v0x + c*v1x + s*v2x;
	y = v0y + c*v1y + s*v2y;
	z = v0z	+ c*v1z + s*v2z;

	return ( *this );
}

VectorR3& VectorR3::Load( const double* v ) {
	x = *v;
	y = *(v+1);
	z = *(v+2);
	return *this;
}

VectorR3& VectorR3::Load( const float* v ) {
	x = *v;
	y = *(v+1);
	z = *(v+2);
	return *this;
}

void VectorR3::Dump(double *v) const {
	*v = x;
	*(v+1) = y;
	*(v+2) = z;
}

void VectorR3::Dump(float *v) const {
	*v = (float)x;
	*(v+1) = (float)y;
	*(v+2) = (float)z;
}

VectorR3& VectorR3::Set(double xx, double yy, double zz) {
	x = xx;
	y = yy;
	z = zz;
	return *this;
}

VectorR3& VectorR3::SetZero(void) {
	x = 0.0;
	y = 0.0;
	z = 0.0;
	return *this;
}

// Convert to unit vector (or leave zero)
VectorR3& VectorR3::MakeUnit ()	{
	double nSq = NormSq();
	if (nSq != 0.0) {
		*this /= sqrt(nSq);
	}
	return *this;
}

// Component-wise Product
VectorR3& VectorR3::ArrayProd (const VectorR3& v) {
	x *= v.x;
	y *= v.y;
	z *= v.z;
	return ( *this );
}

VectorR3& VectorR3::AddScaled( const VectorR3& u, double s ) {
	x += s*u.x;
	y += s*u.y;
	z += s*u.z;
	return(*this);
}

bool VectorR3::IsZero(void) {
	return ( x==0.0 && y==0.0 && z==0.0 );
}

double VectorR3::Norm(void) {
	return ( (double)sqrt( x*x + y*y + z*z ) );
}

double VectorR3::NormSq(void) {
	return ( x*x + y*y + z*z );
}

VectorR3& VectorR3::ReNormalize() {
	double nSq = NormSq();
	register double mFact = 1.0-0.5*(nSq-1.0);	// Multiplicative factor
	*this *= mFact;
	return *this;
}

double VectorR3::Dist( const VectorR3& u ) {
	return sqrt( DistSq(u) );
}

double VectorR3::DistSq( const VectorR3& u ) {
	return ( (x-u.x)*(x-u.x) + (y-u.y)*(y-u.y) + (z-u.z)*(z-u.z) );
}

VectorR3& VectorR3::Negate(void) {
	x = -x;
	y = -y;
	z = -z;
	return *this;
}

VectorR3& VectorR3::Normalize(void) {
	*this /= Norm();
	return *this;
}

bool VectorR3::IsUnit(void) {
	register double norm = Norm();
	return ( 1.000001>=norm && norm>=0.999999 );
}

bool VectorR3::IsUnit(double tolerance) {
	register double norm = Norm();
	return ( 1.0+tolerance>=norm && norm>=1.0-tolerance );
}

bool VectorR3::NearZero(double tolerance) {
	return( MaxAbs()<=tolerance );
}



/*double VectorR3::operator[] (int i) {
	switch (i) {
		case 0:
			return x;
		case 1:
			return y;
		case 2:
			return z;
		default:
			assert(0);
			return 0.0;
	}
}
VectorR3& VectorR3::operator= (const VectorR3& v) {
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
	return(*this);
}
VectorR3& VectorR3::operator+= (const VectorR3& v) {
	x += v.x;
	y += v.y;
	z += v.z;
	return(*this);
}
VectorR3& VectorR3::operator-= ( const VectorR3& v ) {
	x-=v.x;
	y-=v.y;
	z-=v.z;
	return(*this);
}
VectorR3& VectorR3::operator*= (double m) {
	x*=m;
	y*=m;
	z*=m;
	return(*this);
}
VectorR3& VectorR3::operator/= (double m) {
	register double mInv = 1.0/m;
	x*=mInv;
	y*=mInv;
	z*=mInv;
	return(*this);
}
VectorR3 VectorR3::operator- () const {
	return ( VectorR3(-x, -y, -z) );
}
*/

// ***************************************************************
//  Stream Output Routines										 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
ostream& operator<< ( ostream& os, const VectorR3& u ) {
	return (os << "<" << u.x << "," << u.y << "," << u.z << ">");
}