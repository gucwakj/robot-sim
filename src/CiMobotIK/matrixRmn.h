#ifndef MATRIX_RMN_H_
#define MATRIX_RMN_H_

#include <cmath>
#include <cassert>
#include "vectorRn.h"

class MatrixRmn {
	public:
		MatrixRmn(void);							// Null constructor
		MatrixRmn(long numRows, long numCols);		// Constructor with length
		~MatrixRmn(void);							// Destructor

		void SetSize(long numRows, long numCols);
		void SetZero(void);
		void Set( long i, long j, double val );
		void SetTriple( long i, long j, const VectorR3& u );
		void setHextuple(int i, int j, const VectorR3& u, const VectorR3& v);
		void SetIdentity();
		void SetDiagonalEntries( double d );
		void SetDiagonalEntries( const VectorRn& d );
		void SetSuperDiagonalEntries( double d );
		void SetSuperDiagonalEntries( const VectorRn& d );
		void SetSubDiagonalEntries( double d );
		void SetSubDiagonalEntries( const VectorRn& d );
		void SetColumn(long i, const VectorRn& d );
		void SetRow(long i, const VectorRn& d );
		void SetSequence( const VectorRn& d, long startRow, long startCol, long deltaRow, long deltaCol );

		// Return entry in row i and column j.
		double Get( long i, long j ) const;
		void GetTriple( long i, long j, VectorR3 *retValue ) const;
		long GetNumRows(void) const { return NumRows; }
		long GetNumColumns(void) const { return NumCols; }

		// Use GetPtr to get pointer into the array (efficient)
		// Is friendly in that anyone can change the array contents (be careful!)
		// The entries are in column order!!!
		// Use this with care.  You may call GetRowStride and GetColStride to navigate
		//			within the matrix.  I do not expect these values to ever change.
		const double* GetPtr() const;
		double* GetPtr();
		const double* GetPtr( long i, long j ) const;
		double* GetPtr( long i, long j );
		const double* GetColumnPtr( long j ) const;
		double* GetColumnPtr( long j );
		const double* GetRowPtr( long i ) const;
		double* GetRowPtr( long i );
		long GetRowStride() const { return NumRows; }		// Step size (stride) along a row
		long GetColStride() const { return 1; }				// Step size (stide) along a column

		// Loads matrix in as a sub-matrix.  (i,j) is the base point. Defaults to (0,0).
		// The "Tranpose" versions load the transpose of A.
		void LoadAsSubmatrix( const MatrixRmn& A );
		void LoadAsSubmatrix( long i, long j, const MatrixRmn& A );
		void LoadAsSubmatrixTranspose( const MatrixRmn& A );
		void LoadAsSubmatrixTranspose( long i, long j, const MatrixRmn& A );

		// Norms
		double FrobeniusNormSq() const;
		double FrobeniusNorm() const;

		// Operations on VectorRn's
		void Multiply( const VectorRn& v, VectorRn& result ) const;					// result = (this)*(v)
		void MultiplyTranspose( const VectorRn& v, VectorRn& result ) const;		// Equivalent to mult by row vector on left
		double DotProductColumn( const VectorRn& v, long colNum ) const;			// Returns dot product of v with i-th column

		// Operations on MatrixRmn's
		MatrixRmn& operator*=( double );
		MatrixRmn& operator/=( double d ) { assert(d!=0.0); *this *= (1.0/d); return *this; }
		MatrixRmn& AddScaled( const MatrixRmn& B, double factor );
		MatrixRmn& operator+=( const MatrixRmn& B );
		MatrixRmn& operator-=( const MatrixRmn& B );
		static MatrixRmn& Multiply( const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst );				// Sets dst = A*B.
		static MatrixRmn& MultiplyTranspose( const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst );		// Sets dst = A*(B-tranpose).
		static MatrixRmn& TransposeMultiply( const MatrixRmn& A, const MatrixRmn& B, MatrixRmn& dst );		// Sets dst = (A-transpose)*B.

		// Miscellaneous operation
		MatrixRmn& AddToDiagonal( double d );					// Adds d to each diagonal

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
		void ComputeSVD( MatrixRmn& U, VectorRn& w, MatrixRmn& V ) const;
		bool DebugCheckSVD( const MatrixRmn& U, const VectorRn& w, const MatrixRmn& V ) const;

		// Some useful routines for experts who understand the inner workings of these classes.
		inline static double DotArray( long length, const double* ptrA, long strideA, const double* ptrB, long strideB );
		inline static void CopyArrayScale( long length, const double* from, long fromStride, double *to, long toStride, double scale );
		inline static void AddArrayScale( long length, const double* from, long fromStride, double *to, long toStride, double scale );

	private:
		long NumRows;				// Number of rows
		long NumCols;				// Number of columns
		double *x;					// Array of vector entries - stored in column order
		long AllocSize;				// Allocated size of the x array

		static MatrixRmn WorkMatrix;	// Temporary work matrix
		static MatrixRmn& GetWorkMatrix() { return WorkMatrix; }
		static MatrixRmn& GetWorkMatrix(long numRows, long numCols) { WorkMatrix.SetSize( numRows, numCols ); return WorkMatrix; }

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