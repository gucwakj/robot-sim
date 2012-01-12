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

        bool debugCheckSVD(const MatrixRmn& U, const VectorRn& w, const MatrixRmn& V) const; // check SVD

        double dotProductColumn(const VectorRn& v, int col_num);        // Returns dot product of v with i-th column
        double frobeniusNorm(void);                                     // Frobenius Norm

        void addArrayScale(int length, const double *from, int fromStride, double *to, int toStride, double scale) const;
        void addToDiagonal(double d);                                   // Adds d to each diagonal
        void computeSVD(MatrixRmn& U, VectorRn& w, MatrixRmn& V) const; // Singular value decomposition
        void convertToREF(void);                                        // Converts the matrix in place to row echelon form
        void multiply(const VectorRn& v, VectorRn& result);             // {result} = [this]*{v}
        void multiply(const MatrixRmn& B, MatrixRmn& result) const;     // [result] = [this]*[B]
        void multiplyTranspose(const VectorRn& v, VectorRn& result);    // {result} = {v}*[this]
        void multiplyTranspose(const MatrixRmn& B, MatrixRmn& result) const; // [result] = [this]*[B]^T
        void postApplyGivens(double c, double s, int idx1, int idx2);   // Applies Givens transform to columns idx1 and idx2
        void solve(const VectorRn& b, VectorRn *xVec);                  // Solves the equation [this]*{x} = {b}
        void transposeMultiply(const MatrixRmn& B, MatrixRmn& result) const; // [result] = [this]^T*[B]

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

        bool update_bidiag_indices(int *firstBidiagIdx, int *lastBidiagIdx, VectorRn& w, VectorRn& superDiag, double eps) const;
        double dot_array(int length, const double *ptrA, int strideA, const double *ptrB, int strideB) const;
        int get_diagonal_length(void);
        void apply_givens_cbtd(double cosine, double sine, double *a, double *b, double *c, double *d) const;
        void apply_givens_cbtd(double cosine, double sine, double *a, double *b, double *c, double  d, double *e, double *f) const;
        void calc_bidiagonal(MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag) const;
        void calc_givens_values(double a, double b, double *c, double *s) const;
        void clear_column_with_diagonal_zero(int endIdx, MatrixRmn& V, double *wPtr, double *sdPtr, double eps) const;
        void clear_row_with_diagonal_zero(int firstBidiagIdx, int lastBidiagIdx, MatrixRmn& U, double *wPtr, double *sdPtr, double eps) const;
        void convert_bidiag_to_diag(MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag) const;
        void copy_array_scale(int length, const double *from, int fromStride, double *to, int toStride, double scale) const;
        void expand_householders(int numXforms, int numZerosSkipped, const double *basePt, int colStride, int rowStride);
        void load_as_submatrix(const MatrixRmn& A);
        void load_as_submatrix_transpose(const MatrixRmn& A);
        void svd_householder(double *basePt, int colLength, int numCols, int colStride, int rowStride, double *retFirstEntry) const;
};

#endif	/* MATRIXRMN_H_ */