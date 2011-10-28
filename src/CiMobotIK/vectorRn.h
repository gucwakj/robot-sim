#ifndef VECTOR_RN_H_
#define VECTOR_RN_H_

#include <cmath>
#include <cassert>
#include "vectorR3.h"

class VectorRn {
	friend class MatrixRmn;

	public:
		VectorRn(void);				    // Null constructor
		VectorRn(int init_length);      // Constructor with length
		~VectorRn(void);			    // Destructor

		void set(const double *d, double scale = 1);
        void setLength(int new_length);
        void setValue(double d);
        void setZero(void);
        void setTriplePosition(int i, const VectorR3& u);
        void setTripleRotation(int i, const VectorR3& v);
        void setHextuple(int i, const VectorR3&u, const VectorR3& v);

        int getLength(void) const;
        double* getPtr(void);
        const double* getPtr(void) const;
        double* getPtr(int i);
        const double* getPtr(int i) const;

        void add(const VectorRn& src, double scale = 1);
        double dot(const VectorRn& v);
        double maxAbs(void) const;
        double norm(void);
        double normSq(void);
        double norm(int i);
        double normSq(int i);

        double& operator[] (int i);
        const double& operator[] (int i) const;
		VectorRn& operator+= (const VectorRn& src);
        VectorRn& operator-= (const VectorRn& src);
        VectorRn& operator*= (double f);
	private:
		int m_length;			            // Logical or actual length
		int m_alloc_length;			        // Allocated length
		double *x;					        // Array of vector entries
};

#endif /*VECTOR_RN_H*/