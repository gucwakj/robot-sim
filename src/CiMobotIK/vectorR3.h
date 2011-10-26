#ifndef VECTORR3_H_
#define VECTORR3_H_

#include <cmath>
#include <iostream>

using namespace std;

class VectorR3 {
	public:
		VectorR3(void);
		VectorR3(double xVal, double yVal, double zVal);

        double Dist(const VectorR3& u);                     // Distance from u
        double DistSq(const VectorR3& u);                   // Distance from u squared
        double MaxAbs(void);
		double Norm(void);
		double NormSq(void);
        void Dump(double *v) const;
        void Dump(float *v) const;
		VectorR3& AddScaled(const VectorR3& u, double s);
		VectorR3& ArrayProd(const VectorR3&);				// Component-wise product
		VectorR3& Negate(void);
		VectorR3& Normalize(void);							// No error checking
		VectorR3& MakeUnit(void);							// Normalize() with error checking
		VectorR3& ReNormalize(void);
		VectorR3& Rotate(double theta, const VectorR3& u);	// rotate around u
		VectorR3& Set(double xx, double yy, double zz);
		VectorR3& SetZero(void);
		VectorR3& Load(const double *v);
		VectorR3& Load(const float *v);

		double x, y, z;										// The x & y & z coordinates

		double operator[] (int i);
		VectorR3 operator- (void);
		VectorR3& operator= (const VectorR3& v);
		VectorR3& operator+= (const VectorR3& v);
		VectorR3& operator-= (const VectorR3& v);
		VectorR3& operator*= (double m);
		VectorR3& operator/= (double m);
		VectorR3& operator*= (const VectorR3& v);			// Cross Product

		friend ostream& operator<< (ostream& os, const VectorR3& u);
};

int operator== (const VectorR3& u, const VectorR3& v);
VectorR3 operator* (double m, const VectorR3& u);
VectorR3 operator* (const VectorR3& u, double m);
VectorR3 operator/ (const VectorR3& u, double m);
VectorR3 operator+ (const VectorR3& u, const VectorR3& v);
VectorR3 operator- (const VectorR3& u, const VectorR3& v);
VectorR3 operator* (const VectorR3& u, const VectorR3& v);	// Cross Product

ostream& operator<< (ostream& os, const VectorR3& u);

#endif	/* VECTORR3_H_ */