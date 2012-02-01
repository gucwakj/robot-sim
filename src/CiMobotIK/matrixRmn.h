#ifndef MATRIXRMN_H_
#define MATRIXRMN_H_

#include <cmath>
#include <cassert>
#include "vectorRn.h"

class MatrixRmn {
	public:
		MatrixRmn(void);							// Null constructor
        MatrixRmn(int num_row, int num_col);		// Constructor with length
		~MatrixRmn(void);							// Destructor

        void setColumn(int i, VectorRn& d);
        void setDiagonalEntries(VectorRn& d);
        void setIdentity(void);
        void setSequence(VectorRn& d, int start_row, int start_col, int delta_row, int delta_col);
        void setSize(int num_row, int num_col);
        void setTriplePosition(int i, int j, const VectorR3& u);
        void setTripleRotation(int i, int j, const VectorR3& v);
        void setZero(void);

        int getNumRows(void);
        int getNumColumns(void);
        double* getPtr(void);
        double* getPtr(int i, int j);
        double* getColumnPtr(int j);

        bool debugCheckSVD(MatrixRmn& U, VectorRn& w, MatrixRmn& V);    // check SVD
        double dotProductColumn(VectorRn& v, int col_num);              // Returns dot product of v with i-th column
        double frobeniusNorm(void);                                     // Frobenius Norm
        double norm(int i);                                             // norm
        void addArrayScale(int length, double *from, int fromStride, double *to, int toStride, double scale);
        void addToDiagonal(double d);                                   // Adds d to each diagonal
        void computeSVD(MatrixRmn& U, VectorRn& w, MatrixRmn& V);       // Singular value decomposition
        void convertToREF(void);                                        // Converts the matrix in place to row echelon form
        void multiply(VectorRn& v, VectorRn& result);                   // {result} = [this]*{v}
        void multiply(MatrixRmn& B, MatrixRmn& result);                 // [result] = [this]*[B]
        void multiplyTranspose(VectorRn& v, VectorRn& result);          // {result} = {v}*[this]
        void multiplyTranspose(MatrixRmn& B, MatrixRmn& result);        // [result] = [this]*[B]^T
        void postApplyGivens(double c, double s, int idx1, int idx2);   // Applies Givens transform to columns idx1 and idx2
        void solve(VectorRn& b, VectorRn *xVec);                        // Solves the equation [this]*{x} = {b}
        void transposeMultiply(MatrixRmn& B, MatrixRmn& result);        // [result] = [this]^T*[B]

        MatrixRmn& operator*= (double d);
        MatrixRmn& operator/= (double d);
        MatrixRmn& operator+= (MatrixRmn& B);
        MatrixRmn& operator-= (MatrixRmn& B);
    private:
        int m_alloc_size;           // Allocated size of the x array
        int m_num_rows;				// Number of rows
        int m_num_cols;				// Number of columns
        int m_size;                 // Current size of array
        double *x;					// Array of vector entries - stored in column order

        bool update_bidiag_indices(int *firstBidiagIdx, int *lastBidiagIdx, VectorRn& w, VectorRn& superDiag, double eps);
        double dot_array(int length, double *ptrA, int strideA, double *ptrB, int strideB);
        int get_diagonal_length(void);
        void apply_givens_cbtd(double cosine, double sine, double *a, double *b, double *c, double *d);
        void apply_givens_cbtd(double cosine, double sine, double *a, double *b, double *c, double  d, double *e, double *f);
        void calc_bidiagonal(MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag);
        void calc_givens_values(double a, double b, double *c, double *s);
        void clear_column_with_diagonal_zero(int endIdx, MatrixRmn& V, double *wPtr, double *sdPtr, double eps);
        void clear_row_with_diagonal_zero(int firstBidiagIdx, int lastBidiagIdx, MatrixRmn& U, double *wPtr, double *sdPtr, double eps);
        void convert_bidiag_to_diag(MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag);
        void copy_array_scale(int length, double *from, int fromStride, double *to, int toStride, double scale);
        void expand_householders(int numXforms, int numZerosSkipped, double *basePt, int colStride, int rowStride);
        void load_as_submatrix(MatrixRmn& A);
        void load_as_submatrix_transpose(MatrixRmn& A);
        void svd_householder(double *basePt, int colLength, int numCols, int colStride, int rowStride, double *retFirstEntry);
};

#endif	/* MATRIXRMN_H_ */