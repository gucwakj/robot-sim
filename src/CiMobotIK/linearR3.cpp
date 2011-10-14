#include "linearR3.h"

// ******************************************************
// * VectorR3 class - math library functions			*
// ******************************************************
const VectorR3 UnitVecIR3(1.0, 0.0, 0.0);
const VectorR3 UnitVecJR3(0.0, 1.0, 0.0);
const VectorR3 UnitVecKR3(0.0, 0.0, 1.0);

const VectorR3 VectorR3::Zero(0.0, 0.0, 0.0);
const VectorR3 VectorR3::UnitX( 1.0, 0.0, 0.0);
const VectorR3 VectorR3::UnitY( 0.0, 1.0, 0.0);
const VectorR3 VectorR3::UnitZ( 0.0, 0.0, 1.0);
const VectorR3 VectorR3::NegUnitX(-1.0, 0.0, 0.0);
const VectorR3 VectorR3::NegUnitY( 0.0,-1.0, 0.0);
const VectorR3 VectorR3::NegUnitZ( 0.0, 0.0,-1.0);

double VectorR3::MaxAbs() const {
	register double m;
	m = (x>0.0) ? x : -x;
	if ( y>m ) m=y;
	else if ( -y >m ) m = -y;
	if ( z>m ) m=z;
	else if ( -z>m ) m = -z;
	return m;
}

// *********************************************************************
// Rotation routines												   *
// *********************************************************************

// s.Rotate(theta, u) rotates s and returns s
//        rotated theta degrees around unit vector w.
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

// Rotate unit vector x in the direction of "dir": length of dir is rotation angle.
//		x must be a unit vector.  dir must be perpindicular to x.
VectorR3& VectorR3::RotateUnitInDirection ( const VectorR3& dir) {
	double theta = dir.NormSq();
	if ( theta==0.0 ) {
		return *this;
	}
	else {
		theta = sqrt(theta);
		double costheta = cos(theta);
		double sintheta = sin(theta);
		VectorR3 dirUnit = dir/theta;
		*this = costheta*(*this) + sintheta*dirUnit;
		return ( *this );
	}
}

// ***************************************************************
//  Stream Output Routines										 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
ostream& operator<< ( ostream& os, const VectorR3& u ) {
	return (os << "<" << u.x << "," << u.y << "," << u.z << ">");
}