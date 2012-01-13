#ifndef VECTORR3_H_
#define VECTORR3_H_

#include <cmath>
#include <iostream>
using namespace std;

class VectorR3 {
	public:
		VectorR3(void);
        VectorR3(double x, double y, double z);

        void set(double x, double y, double z);         // set three values for vector

		double norm(void);                              // norm of vector
        void dump(double *v) const;                     // dump values into x,y,z
		void rotate(double theta, VectorR3& u);	        // rotate around u by theta

		double x, y, z;									// The x & y & z coordinates

		double operator[] (int i);
		VectorR3 operator- (void);
		VectorR3& operator= (const VectorR3& v);
		VectorR3& operator+= (const VectorR3& v);
		VectorR3& operator-= (const VectorR3& v);
		VectorR3& operator*= (double m);
		VectorR3& operator/= (double m);
        VectorR3& operator*= (const VectorR3& v);			// Cross Product
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