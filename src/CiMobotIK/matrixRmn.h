#ifndef MATRIX_RMN_H_
#define MATRIX_RMN_H_

#include <cmath>
#include <cassert>
#include "vectorRn.h"

class MatrixRmn {
	public:
		MatrixRmn(void);							// Null constructor
        MatrixRmn(int num_row, int num_col);		// Constructor with length
		~MatrixRmn(void);							// Destructor

        void setColumn(int i, const VectorRn& d );
        void setDiagonalEntries(const VectorRn& d);
        void setIdentity(void);
        void setSequence(const VectorRn& d, int start_row, int start_col, int delta_row, int delta_col);
        void setSize(int num_row, int num_col);
        void setTriplePosition(int i, int j, const VectorR3& u);
        void setTripleRotation(int i, int j, const VectorR3& v);
        void setZero(void);

        int getNumRows(void) const;
        int getNumColumns(void) const;
        double* getPtr(void);
        const double* getPtr(void) const;
        double* getPtr(int i, int j);
        const double* getPtr(int i, int j) const;
        double* getColumnPtr(int j);
        const double* getColumnPtr(int j) const;

		double frobeniusNorm(void) const;                               // Frobenius Norm

		// Operations on VectorRn's
		void Multiply( const VectorRn& v, VectorRn& result ) const;					// result = (this)*(v)
		void MultiplyTranspose( const VectorRn& v, VectorRn& result ) const;		// Equivalent to mult by row vector on left
		double DotProductColumn( const VectorRn& v, long colNum ) const;			// Returns dot product of v with i-th column

		// Operations on MatrixRmn's
		//MatrixRmn& AddScaled( const MatrixRmn& B, double factor );
		static MatrixRmn& Multiply( const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst );				// Sets dst = A*B.
		static MatrixRmn& MultiplyTranspose( const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst );		// Sets dst = A*(B-tranpose).
		static MatrixRmn& TransposeMultiply( const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst );		// Sets dst = (A-transpose)*B.

		// Miscellaneous operation
		MatrixRmn& addToDiagonal(double d);					// Adds d to each diagonal

		// Solving systems of linear equations
		void Solve( const VectorRn& b, VectorRn* x ) const;	  // Solves the equation   (*this)*x = b;    Uses row operations.  Assumes *this is invertible.

		// Row Echelon Form and Reduced Row Echelon Form routines
		// Row echelon form here allows non-negative entries (instead of 1's) in the positions of lead variables.
		void ConvertToRefNoFree();				// Converts the matrix in place to row echelon form -- assumption is no free variables will be found
		void ConvertToRef( int numVars);		// Converts the matrix in place to row echelon form -- numVars is number of columns to work with.
		void ConvertToRef( int numVars, double eps);		// Same, but eps is the measure of closeness to zero

		// Givens transformation
		static void CalcGivensValues( double a, double b, double *c, double *s );
		void PostApplyGivens( double c, double s, long idx );							// Applies Givens transform to columns idx and idx+1.
		void PostApplyGivens( double c, double s, long idx1, long idx2 );				// Applies Givens transform to columns idx1 and idx2.

		// Singular value decomposition
		void computeSVD(MatrixRmn& U, VectorRn& w, MatrixRmn& V) const;
		bool DebugCheckSVD(const MatrixRmn& U, const VectorRn& w, const MatrixRmn& V) const;

		// Some useful routines for experts who understand the inner workings of these classes.
		inline static double DotArray( long length, const double* ptrA, long strideA, const double* ptrB, long strideB );
		inline static void CopyArrayScale( long length, const double* from, long fromStride, double *to, long toStride, double scale );
		inline static void AddArrayScale( long length, const double* from, long fromStride, double *to, long toStride, double scale );

        MatrixRmn& operator*= (double d);
        MatrixRmn& operator/= (double d);
        MatrixRmn& operator+= (const MatrixRmn& B);
        MatrixRmn& operator-= (const MatrixRmn& B);
    private:
        int m_alloc_size;           // Allocated size of the x array
        int m_num_rows;				// Number of rows
        int m_num_cols;				// Number of columns
        int m_size;                 // Current size of array
        double *x;					// Array of vector entries - stored in column order

		// Internal helper routines for SVD calculations
		static void CalcBidiagonal( MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag );
		void ConvertBidiagToDiagonal( MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag ) const;
		static void SvdHouseholder( double *basePt, long colLength, long numCols, long colStride, long rowStride, double *retFirstEntry);
		void ExpandHouseholders( long numXforms, int numZerosSkipped, const double* basePt, long colStride, long rowStride );
		static bool UpdateBidiagIndices( long *firstDiagIdx, long *lastBidiagIdx, VectorRn& w, VectorRn& superDiag, double eps );
		static void ApplyGivensCBTD( double cosine, double sine, double *a, double *b, double *c, double *d );
		static void ApplyGivensCBTD( double cosine, double sine, double *a, double *b, double *c, double  d, double *e, double *f );
		static void ClearRowWithDiagonalZero( long firstBidiagIdx, long lastBidiagIdx, MatrixRmn& U, double *wPtr, double *sdPtr, double eps );
		static void ClearColumnWithDiagonalZero( long endIdx, MatrixRmn& V, double *wPtr, double *sdPtr, double eps );
		bool DebugCalcBidiagCheck( const MatrixRmn& U, const VectorRn& w, const VectorRn& superDiag, const MatrixRmn& V ) const;

        void load_as_submatrix(const MatrixRmn& A);
        void load_as_submatrix_transpose(const MatrixRmn& A);
};

// Helper routine to calculate dot product
inline double MatrixRmn::DotArray( long length, const double* ptrA, long strideA, const double* ptrB, long strideB )
{
	double result = 0.0;
    for ( ; length>0 ; length-- ) {
		result += (*ptrA)*(*ptrB);
		ptrA += strideA;
		ptrB += strideB;
	}
	return result;
}

// Helper routine: copies and scales an array (src and dest may be equal, or overlap)
inline void MatrixRmn::CopyArrayScale( long length, const double* from, long fromStride, double *to, long toStride, double scale )
{
	for ( ; length>0; length-- ) {
		*to = (*from)*scale;
		from += fromStride;
		to += toStride;
	}
}

// Helper routine: adds a scaled array
//	fromArray = toArray*scale.
inline void MatrixRmn::AddArrayScale( long length, const double* from, long fromStride, double *to, long toStride, double scale )
{
	for ( ; length>0; length-- ) {
		*to += (*from)*scale;
		from += fromStride;
		to += toStride;
	}
}

#endif	/* MATRIXRMN_H_ */