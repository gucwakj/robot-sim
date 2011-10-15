#ifndef LINEAR_R3_H_
#define LINEAR_R3_H_

#include <cmath>
#include <cassert>
#include <iostream>

using namespace std;

class VectorR3 {
	public:
		//VectorR3( ) : x(0.0), y(0.0), z(0.0) {}
		//VectorR3( double xVal, double yVal, double zVal )
		//	: x(xVal), y(yVal), z(zVal) {}
		VectorR3(void);
		VectorR3(double xVal, double yVal, double zVal);

		bool IsZero(void);
		bool IsUnit(void);
		bool IsUnit(double tolerance);
		bool NearZero(double tolerance);
		double Norm(void);
		double NormSq(void);
		double MaxAbs(void);
		double Dist( const VectorR3& u );					// Distance from u
		double DistSq( const VectorR3& u );					// Distance from u squared
		VectorR3& AddScaled(const VectorR3& u, double s);
		VectorR3& ArrayProd(const VectorR3&);				// Component-wise product
		VectorR3& Negate(void);
		VectorR3& Normalize(void);							// No error checking
		VectorR3& MakeUnit();								// Normalize() with error checking
		VectorR3& ReNormalize();
		VectorR3& Rotate(double theta, const VectorR3& u);	// rotate around u
		VectorR3& Set(double xx, double yy, double zz);
		VectorR3& SetZero(void);
		VectorR3& Load(const double *v);
		VectorR3& Load(const float *v);
		void Dump(double *v) const;
		void Dump(float *v) const;

		double x, y, z;		// The x & y & z coordinates

		inline double operator[]( int i );
		VectorR3& operator= ( const VectorR3& v )
			{ x=v.x; y=v.y; z=v.z; return(*this);}
		VectorR3& operator+= ( const VectorR3& v )
			{ x+=v.x; y+=v.y; z+=v.z; return(*this); }
		VectorR3& operator-= ( const VectorR3& v )
			{ x-=v.x; y-=v.y; z-=v.z; return(*this); }
		VectorR3& operator*= ( double m )
			{ x*=m; y*=m; z*=m; return(*this); }
		VectorR3& operator/= ( double m )
				{ register double mInv = 1.0/m;
				x*=mInv; y*=mInv; z*=mInv;
				return(*this); }
		VectorR3 operator- () const { return ( VectorR3(-x, -y, -z) ); }
		VectorR3& operator*= (const VectorR3& v);	// Cross Product
		friend ostream& operator<< ( ostream& os, const VectorR3& u );
};

inline VectorR3 operator+( const VectorR3& u, const VectorR3& v );
inline VectorR3 operator-( const VectorR3& u, const VectorR3& v );
inline VectorR3 operator*( const VectorR3& u, double m);
inline VectorR3 operator*( double m, const VectorR3& u);
inline VectorR3 operator/( const VectorR3& u, double m);
inline int operator==( const VectorR3& u, const VectorR3& v );
inline double operator^ (const VectorR3& u, const VectorR3& v ); // Dot Product
inline VectorR3 operator* (const VectorR3& u, const VectorR3& v);	 // Cross Product

ostream& operator<< ( ostream& os, const VectorR3& u );

// *****************************************************
// * VectorR3 class - inlined functions				   *
// * * * * * * * * * * * * * * * * * * * * * * * * * * *
inline double VectorR3::operator[]( int i ) {
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
inline VectorR3 operator+( const VectorR3& u, const VectorR3& v ) {
	return VectorR3(u.x+v.x, u.y+v.y, u.z+v.z);
}
inline VectorR3 operator-( const VectorR3& u, const VectorR3& v ) {
	return VectorR3(u.x-v.x, u.y-v.y, u.z-v.z);
}
inline VectorR3 operator*( const VectorR3& u, register double m) {
	return VectorR3( u.x*m, u.y*m, u.z*m);
}
inline VectorR3 operator*( register double m, const VectorR3& u) {
	return VectorR3( u.x*m, u.y*m, u.z*m);
}
inline VectorR3 operator/( const VectorR3& u, double m) {
	register double mInv = 1.0/m;
	return VectorR3( u.x*mInv, u.y*mInv, u.z*mInv);
}
inline int operator==( const VectorR3& u, const VectorR3& v ) {
	return ( u.x==v.x && u.y==v.y && u.z==v.z );
}
inline double operator^ ( const VectorR3& u, const VectorR3& v ) {
	return ( u.x*v.x + u.y*v.y + u.z*v.z );
}
inline VectorR3 operator* (const VectorR3& u, const VectorR3& v) {
	return (VectorR3(	u.y*v.z - u.z*v.y,
						u.z*v.x - u.x*v.z,
						u.x*v.y - u.y*v.x  ) );
}
inline VectorR3& VectorR3::operator*= (const VectorR3& v) {
	double tx=x, ty=y;
	x =  y*v.z -  z*v.y;
	y =  z*v.x - tx*v.z;
	z = tx*v.y - ty*v.x;
	return ( *this );
}

#endif	/* LINEAR_R3_H_ */