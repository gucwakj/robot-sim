#ifndef MATRIXR33_H_
#define MATRIXR33_H_

#include <cfloat>
#include "vectorR3.h"

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
        VectorR3 getEulerAngles(void);

		void transpose(void);
		void transform(VectorR3 *u);
		void transform(VectorR3& src, VectorR3 *dest);
		MatrixR33& reNormalize(void);
        MatrixR33& rotate(double theta, VectorR3& w);

        MatrixR33& operator*= (MatrixR33& A);

		double m11, m12, m13, m21, m22, m23, m31, m32, m33;
		double psi;
		double theta;
		double phi;
    private:
        void recompute_euler_angles(void);
};

VectorR3 operator* (MatrixR33& A, const VectorR3& u);

#endif	/* MATRIXR33_H_ */