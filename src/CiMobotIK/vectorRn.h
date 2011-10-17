#ifndef VECTOR_RN_H_
#define VECTOR_RN_H_

#include <cmath>
#include <cassert>
#include "vectorR3.h"

class VectorRn {
	friend class MatrixRmn;

	public:
		VectorRn();					// Null constructor
		VectorRn( long length );	// Constructor with length
		~VectorRn();				// Destructor

		void SetLength( long newLength );
		long GetLength() const { return length; }

		void SetZero();
		void Fill( double d );
		void Load( const double* d );
		void LoadScaled( const double* d, double scaleFactor );
		void Set ( const VectorRn& src );

		// Two access methods identical in functionality
		// Subscripts are ZERO-BASED!!
		const double& operator[]( long i ) const { assert ( 0<=i && i<length );	return *(x+i); }
		double& operator[]( long i ) { assert ( 0<=i && i<length );	return *(x+i); }
		double Get( long i ) const { assert ( 0<=i && i<length );	return *(x+i); }

		// Use GetPtr to get pointer into the array (efficient)
		// Is friendly in that anyone can change the array contents (be careful!)
		const double* GetPtr( long i ) const { assert(0<=i && i<length);  return (x+i); }
		double* GetPtr( long i ) { assert(0<=i && i<length);  return (x+i); }
		const double* GetPtr() const { return x; }
		double* GetPtr() { return x; }

		void Set( long i, double val ) { assert(0<=i && i<length), *(x+i) = val; }
		void SetTriple( long i, const VectorR3& u );

		VectorRn& operator+=( const VectorRn& src );
		VectorRn& operator-=( const VectorRn& src );
		void AddScaled (const VectorRn& src, double scaleFactor );

		VectorRn& operator*=( double f );
		double NormSq() const;
		double Norm() const { return sqrt(NormSq()); }

		double MaxAbs() const;
	private:
		long length;				// Logical or actual length
		long AllocLength;			// Allocated length
		double *x;					// Array of vector entries

		static VectorRn WorkVector;								// Serves as a temporary vector
		static VectorRn& GetWorkVector() { return WorkVector; }
		static VectorRn& GetWorkVector( long len ) { WorkVector.SetLength(len); return WorkVector; }
};

inline double Dot( const VectorRn& u, const VectorRn& v ) {
	assert ( u.GetLength() == v.GetLength() );
	double res = 0.0;
	const double* p = u.GetPtr();
	const double* q = v.GetPtr();
	for ( long i = u.GetLength(); i>0; i-- ) {
		res += (*(p++))*(*(q++));
	}
	return res;
}

#endif /*VECTOR_RN_H*/