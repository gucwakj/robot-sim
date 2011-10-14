#ifndef LINEAR_R3_H_
#define LINEAR_R3_H_

#include <cmath>
#include <cassert>
#include <iostream>

using namespace std;

class VectorR3 {
	public:
		double x, y, z;		// The x & y & z coordinates.

		static const VectorR3 Zero;
		static const VectorR3 UnitX;
		static const VectorR3 UnitY;
		static const VectorR3 UnitZ;
		static const VectorR3 NegUnitX;
		static const VectorR3 NegUnitY;
		static const VectorR3 NegUnitZ;
	public:
		VectorR3( ) : x(0.0), y(0.0), z(0.0) {}
		VectorR3( double xVal, double yVal, double zVal )
			: x(xVal), y(yVal), z(zVal) {}

		VectorR3& Set( double xx, double yy, double zz ){ x=xx; y=yy; z=zz; return *this; }
		VectorR3& SetZero() { x=0.0; y=0.0; z=0.0;  return *this;}
		VectorR3& Load( const double* v );
		VectorR3& Load( const float* v );
		void Dump( double* v ) const;
		void Dump( float* v ) const;

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
		VectorR3& ArrayProd(const VectorR3&);		// Component-wise product

		VectorR3& AddScaled( const VectorR3& u, double s );

		bool IsZero() const { return ( x==0.0 && y==0.0 && z==0.0 ); }
		double Norm() const { return ( (double)sqrt( x*x + y*y + z*z ) ); }
		double NormSq() const { return ( x*x + y*y + z*z ); }
		double MaxAbs() const;
		double Dist( const VectorR3& u ) const;	// Distance from u
		double DistSq( const VectorR3& u ) const;	// Distance from u squared
		VectorR3& Negate() { x = -x; y = -y; z = -z; return *this;}
		VectorR3& Normalize () { *this /= Norm(); return *this;}	// No error checking
		inline VectorR3& MakeUnit();		// Normalize() with error checking
		inline VectorR3& ReNormalize();
		bool IsUnit( ) const
			{ register double norm = Norm();
			return ( 1.000001>=norm && norm>=0.999999 ); }
		bool IsUnit( double tolerance ) const
			{ register double norm = Norm();
			return ( 1.0+tolerance>=norm && norm>=1.0-tolerance ); }
		bool NearZero(double tolerance) const { return( MaxAbs()<=tolerance );}

		double YaxisDistSq() const { return (x*x+z*z); }
		double YaxisDist() const { return sqrt(x*x+z*z); }

		VectorR3& Rotate( double theta, const VectorR3& u); // rotate around u.
		VectorR3& RotateUnitInDirection ( const VectorR3& dir);	// rotate in direction dir

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
inline VectorR3 ArrayProd ( const VectorR3& u, const VectorR3& v );

inline double Mag(const VectorR3& u) { return u.Norm(); }
inline double Dist(const VectorR3& u, const VectorR3& v) { return u.Dist(v); }
inline double DistSq(const VectorR3& u, const VectorR3& v) { return u.DistSq(v); }
inline double NormalizeError (const VectorR3& u);

extern const VectorR3 UnitVecIR3;
extern const VectorR3 UnitVecJR3;
extern const VectorR3 UnitVecKR3;

VectorR3 Interpolate( const VectorR3& start, const VectorR3& end, double a);

ostream& operator<< ( ostream& os, const VectorR3& u );

// *****************************************************
// * VectorR3 class - inlined functions				   *
// * * * * * * * * * * * * * * * * * * * * * * * * * * *
inline VectorR3& VectorR3::Load( const double* v )
{
	x = *v;
	y = *(v+1);
	z = *(v+2);
	return *this;
}

inline VectorR3& VectorR3::Load( const float* v )
{
	x = *v;
	y = *(v+1);
	z = *(v+2);
	return *this;
}

inline 	void VectorR3::Dump( double* v ) const
{
	*v = x;
	*(v+1) = y;
	*(v+2) = z;
}

inline 	void VectorR3::Dump( float* v ) const
{
	*v = (float)x;
	*(v+1) = (float)y;
	*(v+2) = (float)z;
}

inline double VectorR3::operator[]( int i )
{
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

inline VectorR3& VectorR3::MakeUnit ()			// Convert to unit vector (or leave zero).
{
	double nSq = NormSq();
	if (nSq != 0.0) {
		*this /= sqrt(nSq);
	}
	return *this;
}

inline VectorR3 operator+( const VectorR3& u, const VectorR3& v )
{
	return VectorR3(u.x+v.x, u.y+v.y, u.z+v.z);
}
inline VectorR3 operator-( const VectorR3& u, const VectorR3& v )
{
	return VectorR3(u.x-v.x, u.y-v.y, u.z-v.z);
}
inline VectorR3 operator*( const VectorR3& u, register double m)
{
	return VectorR3( u.x*m, u.y*m, u.z*m);
}
inline VectorR3 operator*( register double m, const VectorR3& u)
{
	return VectorR3( u.x*m, u.y*m, u.z*m);
}
inline VectorR3 operator/( const VectorR3& u, double m)
{
	register double mInv = 1.0/m;
	return VectorR3( u.x*mInv, u.y*mInv, u.z*mInv);
}

inline int operator==( const VectorR3& u, const VectorR3& v )
{
	return ( u.x==v.x && u.y==v.y && u.z==v.z );
}

inline double operator^ ( const VectorR3& u, const VectorR3& v ) // Dot Product
{
	return ( u.x*v.x + u.y*v.y + u.z*v.z );
}

inline VectorR3 operator* (const VectorR3& u, const VectorR3& v)	// Cross Product
{
	return (VectorR3(	u.y*v.z - u.z*v.y,
					u.z*v.x - u.x*v.z,
					u.x*v.y - u.y*v.x  ) );
}

inline VectorR3 ArrayProd ( const VectorR3& u, const VectorR3& v )
{
	return ( VectorR3( u.x*v.x, u.y*v.y, u.z*v.z ) );
}

inline VectorR3& VectorR3::operator*= (const VectorR3& v)		// Cross Product
{
	double tx=x, ty=y;
	x =  y*v.z -  z*v.y;
	y =  z*v.x - tx*v.z;
	z = tx*v.y - ty*v.x;
	return ( *this );
}

inline VectorR3& VectorR3::ArrayProd (const VectorR3& v)		// Component-wise Product
{
	x *= v.x;
	y *= v.y;
	z *= v.z;
	return ( *this );
}

inline VectorR3& VectorR3::AddScaled( const VectorR3& u, double s )
{
	x += s*u.x;
	y += s*u.y;
	z += s*u.z;
	return(*this);
}

inline VectorR3& VectorR3::ReNormalize()			// Convert near unit back to unit
{
	double nSq = NormSq();
	register double mFact = 1.0-0.5*(nSq-1.0);	// Multiplicative factor
	*this *= mFact;
	return *this;
}

inline double NormalizeError (const VectorR3& u)
{
	register double discrepancy;
	discrepancy = u.x*u.x + u.y*u.y + u.z*u.z - 1.0;
	if ( discrepancy < 0.0 ) {
		discrepancy = -discrepancy;
	}
	return discrepancy;
}

inline double VectorR3::Dist( const VectorR3& u ) const 	// Distance from u
{
	return sqrt( DistSq(u) );
}

inline double VectorR3::DistSq( const VectorR3& u ) const	// Distance from u
{
	return ( (x-u.x)*(x-u.x) + (y-u.y)*(y-u.y) + (z-u.z)*(z-u.z) );
}

// Interpolate(start,end,frac) - linear interpolation
//		- allows overshooting the end points
inline VectorR3 Interpolate( const VectorR3& start, const VectorR3& end, double a)
{
	VectorR3 ret;
	//Lerp( start, end, a, ret );
	double beta = 1.0-a;
	if ( beta > a ) {
		ret = end;
		ret *= a/beta;
		ret += start;
		ret *= beta;
	}
	else {
		ret = start;
		ret *= beta/a;
		ret += end;
		ret *= a;
	}
	return ret;
}

#endif	/* LINEAR_R3_H_ */