#ifndef MATRIXR33_H_
#define MATRIXR33_H_

#include <cmath>
#include <iostream>
#include "vectorR3.h"

using namespace std;

class MatrixR33 {
	public:
		MatrixR33(void);
        MatrixR33(double psi, double theta, double phi);
		MatrixR33(double a11, double a21, double a31, double a12, double a22, double a32, double a13, double a23, double a33);

        void set(double psi, double theta, double phi);
		void set(double a11, double a21, double a31, double a12, double a22, double a32, double a13, double a23, double a33);
		void setColumn1(double a11, double a21, double a31);
		void setColumn2(double a12, double a22, double a32);
		void setColumn3(double a13, double a23, double a33);
		void setIdentity(void);
		void setZero(void);

		VectorR3 getColumn1(void);
		VectorR3 getColumn2(void);
		VectorR3 getColumn3(void);

		void transpose(void);
		void transform(VectorR3 *u);
		void transform(const VectorR3& src, VectorR3 *dest);
		MatrixR33& reNormalize(void);
        MatrixR33& rotate(double theta, const VectorR3& w);

		double m11, m12, m13, m21, m22, m23, m31, m32, m33;
		double psi;
		double theta;
		double phi;
};

inline VectorR3 operator* (const MatrixR33& A, const VectorR3& u);
ostream& operator<< ( ostream& os, const MatrixR33& A );

inline VectorR3 operator* (const MatrixR33& A, const VectorR3& u) {
	return( VectorR3(   A.m11*u.x + A.m12*u.y + A.m13*u.z,
                        A.m21*u.x + A.m22*u.y + A.m23*u.z,
                        A.m31*u.x + A.m32*u.y + A.m33*u.z) );
}

#endif	/* MATRIXR33_H_ */