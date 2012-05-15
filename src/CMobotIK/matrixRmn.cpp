#include "matrixRmn.h"

MatrixRmn::MatrixRmn(void) {
    this->m_alloc_size = 0;
    this->m_num_rows = 0;
    this->m_num_cols = 0;
    this->m_size = 0;
    this->x = 0;
}

MatrixRmn::MatrixRmn(int num_row, int num_col) {
    this->m_alloc_size = 0;
	this->m_num_rows = 0;
    this->m_num_cols = 0;
    this->m_size = 0;
    this->x = 0;
	this->setSize(num_row, num_col);
    this->setZero();
}

MatrixRmn::~MatrixRmn(void) {
	delete this->x;
}

void MatrixRmn::setColumn(int i, VectorRn& d) {
    assert ( m_num_rows==d.getLength() );
    double *to = this->x + i*this->m_num_rows;
    double *from = d.x;
    for ( i = this->m_num_rows; i > 0; i-- ) {
        *(to++) = *(from++);
    }
}

// Fill the diagonal entries with values in vector d.  The rest of the matrix is unchanged.
void MatrixRmn::setDiagonalEntries(VectorRn& d) {
    int diag_len = get_diagonal_length();
    assert ( d.getLength() == diag_len );
    double *to = this->x;
    double *from = d.x;
    for ( ; diag_len > 0; diag_len-- ) {
        *to = *(from++);
        to += this->m_num_rows + 1;
    }
}

void MatrixRmn::setIdentity(void) {
    assert ( this->m_num_rows == this->m_num_cols );
    this->setZero();
    double *dPtr = this->x;
    for ( int diag_len = this->m_num_rows; diag_len > 0; diag_len-- ) {
        *dPtr = 1;
        dPtr += this->m_num_rows + 1;
    }
}

// Sets a "linear" portion of the array with the values from a vector d
void MatrixRmn::setSequence(VectorRn& d, int start_row, int start_col, int delta_row, int delta_col) {
    int length = d.getLength();

    assert( start_row>=0 && start_row<m_num_rows && start_col>=0 && start_col<m_num_cols );
    assert( start_row+(length-1)*delta_row>=0 && start_row+(length-1)*delta_row<m_num_rows );
    assert( start_col+(length-1)*delta_col>=0 && start_col+(length-1)*delta_col<m_num_cols );

    double *to = this->x + start_row + this->m_num_rows*start_col;
    double *from = d.x;
    int stride = delta_row + this->m_num_rows*delta_col;
    for ( ; length > 0; length-- ) {
        *to = *(from++);
        to += stride;
    }
}

void MatrixRmn::setSize(int num_row, int num_col) {
    assert ( num_row > 0 && num_col > 0 );
    int new_size = num_row*num_col;
    if ( new_size > this->m_alloc_size ) {
        delete x;
        this->m_alloc_size = ((new_size > (this->m_alloc_size<<1)) ? new_size : (this->m_alloc_size<<1));
        x = new double[this->m_alloc_size];
    }
    this->m_num_rows = num_row;
    this->m_num_cols = num_col;
    this->m_size = new_size;
}

void MatrixRmn::setTriplePosition(int i, int j, const VectorR3& u) {
    int step = 6*i;
    assert ( 0<=i && step+5<m_num_rows && 0<=j && j<m_num_cols );
    u.dump(x + j*m_num_rows + step);
}

void MatrixRmn::setTripleRotation(int i, int j, const VectorR3& v) {
    int step = 6*i;
    assert ( 0<=i && step+5<m_num_rows && 0<=j && j<m_num_cols );
    v.dump(x + j*this->m_num_rows + step + 3);
}

void MatrixRmn::setZero(void) {
    double *target = this->x;
    for ( int i = this->m_size; i > 0; i-- ) {
        *(target++) = 0.0;
    }
}

int MatrixRmn::getNumRows(void) {
    return this->m_num_rows;
}

int MatrixRmn::getNumColumns(void) {
    return this->m_num_cols;
}

double* MatrixRmn::getPtr(void) {
    return this->x;
}

double* MatrixRmn::getPtr(int i, int j) {
    assert ( i<m_num_rows && j<m_num_cols );
    return (this->x + j*this->m_num_rows + i);
}

double* MatrixRmn::getColumnPtr(int j) {
    assert ( 0<=j && j<m_num_cols );
    return (this->x + j*this->m_num_rows);
}

bool MatrixRmn::debugCheckSVD(MatrixRmn& U, VectorRn& w, MatrixRmn& V) {
    MatrixRmn IV( V.getNumRows(), V.getNumColumns() );
    IV.setIdentity();
    MatrixRmn VTV( V.getNumRows(), V.getNumColumns() );
    V.transposeMultiply(V, VTV);
    IV -= VTV;
    double error = IV.frobeniusNorm();

    MatrixRmn IU( U.getNumRows(), U.getNumColumns() );
    IU.setIdentity();
    MatrixRmn UTU( U.getNumRows(), U.getNumColumns() );
    U.transposeMultiply(U, UTU);
    IU -= UTU;
    error += IU.frobeniusNorm();

    MatrixRmn Diag( U.getNumRows(), V.getNumRows() );
    Diag.setZero();
    Diag.setDiagonalEntries(w);
    MatrixRmn B(U.getNumRows(), V.getNumRows() );
    MatrixRmn C(U.getNumRows(), V.getNumRows() );
    U.multiply(Diag, B);
    B.multiplyTranspose(V, C);
    C -= *this;
    error += C.frobeniusNorm();

    bool ret = ( fabs(error)<=1.0e-13*w.maxAbs() );
    assert ( ret );
    return ret;
}

// Form the dot product of a vector v with the i-th column of the array
double MatrixRmn::dotProductColumn(VectorRn& v, int col_num) {
    assert ( v.getLength()==m_num_rows );
    double *ptrC = this->x + col_num*this->m_num_rows;
    double *ptrV = v.x;
    double ret = 0;
    for ( int i = this->m_num_rows; i > 0; i-- ) {
        ret += (*(ptrC++))*(*(ptrV++));
    }
    return ret;
}

// Calculate the Frobenius Norm (square root of sum of squares of entries of the matrix)
double MatrixRmn::frobeniusNorm(void) {
    double *aPtr = this->x;
    double result = 0.0;
    for ( int i = this->m_size; i > 0; i-- ) {
        result += (*(aPtr))*(*(aPtr++));
    }
    return sqrt(result);
}

// Helper routine: adds a scaled array
void MatrixRmn::addArrayScale(int length, double *from, int fromStride, double *to, int toStride, double scale) {
    for ( ; length>0; length-- ) {
        *to += (*from)*scale;
        from += fromStride;
        to += toStride;
    }
}

// Add a constant to each entry on the diagonal
void MatrixRmn::addToDiagonal(double d) {
    int diag_len = get_diagonal_length();
    double *dPtr = this->x;
    for ( ; diag_len > 0; diag_len-- ) {
        *dPtr += d;
        dPtr += this->m_num_rows + 1;
    }
}

/*
 * Singular value decomposition.
 * Return othogonal matrices U and V and diagonal matrix with diagonal w such that
 *     (this) = U * Diag(w) * V^T     (V^T is V-transpose.)
 * Diagonal entries have all non-zero entries before all zero entries, but are not
 *      necessarily sorted.  (Someday, I will write ComputedSortedSVD that handles
 *      sorting the eigenvalues by magnitude.)
 */
void MatrixRmn::computeSVD(MatrixRmn& U, VectorRn& w, MatrixRmn& V) {
    assert(U.m_num_rows==m_num_rows && V.m_num_cols==m_num_cols
    && U.m_num_rows==U.m_num_cols && V.m_num_rows==V.m_num_cols
    && w.getLength() == ((m_num_rows<m_num_cols) ? m_num_rows : m_num_cols) );

    VectorRn superDiag(w.getLength() - 1);

    // Choose larger of U, V to hold intermediate results
    MatrixRmn *leftMatrix, *rightMatrix;
    if ( this->m_num_rows >= this->m_num_cols ) {
        U.load_as_submatrix(*this);               // Copy A into U
        leftMatrix = &U;
        rightMatrix = &V;
    }
    else {
        V.load_as_submatrix_transpose(*this);     // Copy A-transpose into V
        leftMatrix = &V;
        rightMatrix = &U;
    }

    this->calc_bidiagonal(*leftMatrix, *rightMatrix, w, superDiag);
    this->convert_bidiag_to_diag(*leftMatrix, *rightMatrix, w, superDiag);
}

/*
 * Converts the matrix (in place) to row echelon form.  Row echelon form allows
 * any non-zero values, not just 1's, in the position for a lead variable.  The
 * assumption is that no free variable will be found.  Algorithm uses row
 * operations and row pivoting (only).  Augmented matrix is correctly
 * accomodated.  Only the first square part participates in the main work of row
 * operations.
 */
void MatrixRmn::convertToREF(void) {
    long numIters = get_diagonal_length();
    double* rowPtr1 = x;
    long diagStep = m_num_rows+1;
    long lenRowLeft = m_num_cols;
    for ( ; numIters>1; numIters-- ) {
        // Find row with most non-zero entry.
        double* rowPtr2 = rowPtr1;
        double maxAbs = fabs(*rowPtr1);
        double *rowPivot = rowPtr1;
        long i;
        for ( i=numIters-1; i>0; i-- ) {
            double& newMax = *(++rowPivot);
            if ( newMax > maxAbs ) {
                maxAbs = *rowPivot;
                rowPtr2 = rowPivot;
            }
            else if ( -newMax > maxAbs ) {
                maxAbs = -newMax;
                rowPtr2 = rowPivot;
            }
        }
        // Pivot step: Swap the row with highest entry to the current row
        if ( rowPtr1 != rowPtr2 ) {
            double *to = rowPtr1;
            for ( long i=lenRowLeft; i>0; i-- ) {
                double temp = *to;
                *to = *rowPtr2;
                *rowPtr2 = temp;
                to += m_num_rows;
                rowPtr2 += m_num_rows;
            }
        }
        // Subtract this row appropriately from all the lower rows (row operation of type 3)
        rowPtr2 = rowPtr1;
        for ( i=numIters-1; i>0; i-- ) {
            rowPtr2++;
            double* to = rowPtr2;
            double* from = rowPtr1;
            assert( *from != 0.0 );
            double alpha = (*to)/(*from);
            *to = 0.0;
            for ( long j=lenRowLeft-1; j>0; j-- ) {
                to += m_num_rows;
                from += m_num_rows;
                *to -= (*from)*alpha;
            }
        }
        // Update for next iteration of loop
        rowPtr1 += diagStep;
        lenRowLeft--;
    }
}

// {result} = [this]*{v}
void MatrixRmn::multiply(VectorRn& v, VectorRn& result) {
    assert ( v.getLength()==m_num_cols && result.getLength()==m_num_rows );
    double *out = result.getPtr();              // Points to entry in result vector
    double *rowPtr = this->x;                   // Points to beginning of next row in matrix
    for ( int j = this->m_num_rows; j > 0; j-- ) {
        double *in = v.getPtr();
        double  *m = rowPtr++;
        *out = 0;
        for ( int i = m_num_cols; i > 0; i-- ) {
            *out += (*(in++)) * (*m);
            m += this->m_num_rows;
        }
        out++;
    }
}

// [result] = [this]*[B]
void MatrixRmn::multiply(MatrixRmn& B, MatrixRmn& result) {
    assert( this->m_num_cols == B.m_num_rows && this->m_num_rows == result.getNumRows() && B.m_num_cols == result.getNumColumns());
    int length = this->m_num_cols;

    double *bPtr = B.x;                     // Points to beginning of column in B
    double *dPtr = result.x;
    for ( int i = result.getNumColumns(); i > 0; i-- ) {
        double *aPtr = this->x;                 // Points to beginning of row in A
        for ( int j = result.getNumRows(); j > 0; j-- ) {
            *dPtr = dot_array(length, aPtr, this->m_num_rows, bPtr, 1);
            dPtr++;
            aPtr++;
        }
        bPtr += B.getNumRows();
    }
}

// {result} = {v}*[this]
void MatrixRmn::multiplyTranspose(VectorRn& v, VectorRn& result) {
    assert ( v.getLength()==m_num_rows && result.getLength()==m_num_cols );
    double *out = result.getPtr();              // Points to entry in result vector
    double *colPtr = this->x;             // Points to beginning of next column in matrix
    for ( int i = this->m_num_cols; i > 0; i-- ) {
        double *in = v.getPtr();
        *out = 0;
        for ( int j = this->m_num_rows; j > 0; j-- ) {
            *out += (*(in++)) * (*(colPtr++));
        }
        out++;
    }
}

// [result] = [this]*[B]^T
void MatrixRmn::multiplyTranspose(MatrixRmn& B, MatrixRmn& result) {
    assert(this->m_num_cols == B.getNumColumns() && this->m_num_rows == result.getNumRows() && B.getNumRows() == result.getNumColumns());
    int length = this->m_num_cols;

    double *bPtr = B.x;                     // Points to beginning of row in B
    double *dPtr = result.x;
    for ( int i = result.getNumColumns(); i > 0; i-- ) {
        double *aPtr = this->x;                 // Points to beginning of row in A
        for ( int j = result.getNumRows(); j > 0; j-- ) {
            *dPtr = dot_array(length, aPtr, this->m_num_rows, bPtr, B.getNumRows());
            dPtr++;
            aPtr++;
        }
        bPtr++;
    }
}

// Applies Givens transform to columns idx1 and idx2
void MatrixRmn::postApplyGivens(double c, double s, int idx1, int idx2) {
    assert ( idx1!=idx2 && 0<=idx1 && idx1<m_num_cols && 0<=idx2 && idx2<m_num_cols );
    double *colA = this->x + idx1*this->m_num_rows;
    double *colB = this->x + idx2*this->m_num_rows;
    for ( int i = this->m_num_rows; i > 0; i-- ) {
        double temp = *colA;
        *colA = (*colA)*c + (*colB)*s;
        *colB = (*colB)*c - temp*s;
        colA++;
        colB++;
    }
}

// Solves the equation   [this]*{xVec} = {b}
void MatrixRmn::solve(VectorRn& b, VectorRn *xVec) {
    assert ( m_num_rows==m_num_cols && m_num_cols==xVec->getLength() && m_num_rows==b.getLength() );

    // Copy this matrix and b into an Augmented Matrix
    MatrixRmn AugMat(this->m_num_rows, this->m_num_cols + 1);
    AugMat.load_as_submatrix(*this);
    AugMat.setColumn(this->m_num_rows, b);

    // Put into row echelon form with row operations
    AugMat.convertToREF();

    // Solve for x vector values using back substitution
    double *xLast = xVec->x+m_num_rows-1;               // Last entry in xVec
    double *endRow = AugMat.x+this->m_size-1;   // Last entry in the current row of the coefficient part of Augmented Matrix
    double *bPtr = endRow+m_num_rows;               // Last entry in augmented matrix (end of last column, in augmented part)
    for ( long i = m_num_rows; i>0; i-- ) {
        double accum = *(bPtr--);
        // Next loop computes back substitution terms
        double* rowPtr = endRow;                    // Points to entries of the current row for back substitution.
        double* xPtr = xLast;                       // Points to entries in the x vector (also for back substitution)
        for ( long j=m_num_rows-i; j>0; j-- ) {
            accum -= (*rowPtr)*(*(xPtr--));
            rowPtr -= m_num_cols;                       // Previous entry in the row
        }
        assert( *rowPtr != 0.0 );                   // Are not supposed to be any free variables in this matrix
        *xPtr = accum/(*rowPtr);
        endRow--;
    }
}

// [result] = [this]^T*[B]
void MatrixRmn::transposeMultiply(MatrixRmn& B, MatrixRmn& result) {
    assert( this->m_num_rows == B.m_num_rows && this->m_num_cols == result.m_num_rows && B.m_num_cols == result.m_num_cols);
    int length = this->m_num_rows;

    double *bPtr = B.x;                                     // bPtr Points to beginning of column in B
    double *dPtr = result.x;
    for ( int i = result.m_num_cols; i>0; i-- ) {             // Loop over all columns of dst
        double *aPtr = this->x;                                 // aPtr Points to beginning of column in A
        for ( int j = result.m_num_rows; j > 0; j-- ) {         // Loop over all rows of dst
            *dPtr = dot_array(length, aPtr, 1, bPtr, 1 );
            dPtr ++;
            aPtr += this->m_num_rows;
        }
        bPtr += B.m_num_rows;
    }
}

MatrixRmn& MatrixRmn::operator*= (double d) {
	double* aPtr = x;
	for ( long i=this->m_size; i>0; i-- ) {
		(*(aPtr++)) *= d;
	}
	return (*this);
}

MatrixRmn& MatrixRmn::operator/= (double d) {
    assert(d!=0.0);
    *this *= (1.0/d);
    return (*this);
}

MatrixRmn& MatrixRmn::operator+= (MatrixRmn& B) {
	assert (m_num_rows==B.m_num_rows && m_num_cols==B.m_num_cols);
	double* aPtr = x;
	double* bPtr = B.x;
	for ( int i = this->m_size; i > 0; i-- ) {
		(*(aPtr++)) += *(bPtr++);
	}
	return (*this);
}

MatrixRmn& MatrixRmn::operator-= (MatrixRmn& B) {
	assert (m_num_rows==B.m_num_rows && m_num_cols==B.m_num_cols);
	double* aPtr = x;
	double* bPtr = B.x;
	for ( int i = this->m_size; i > 0; i-- ) {
		(*(aPtr++)) -= *(bPtr++);
	}
	return (*this);
}

/*
 * Calculate Bidiagonal
 *      Helper routine for SVD computation.  U is a matrix to be bidiagonalized.
 *      On return, U and V are orthonormal and w holds the new diagonal elements
 *      and superDiag holds the super diagonal elements.
 */
void MatrixRmn::calc_bidiagonal(MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag) {
	assert ( U.m_num_rows>=V.m_num_rows );

	// The diagonal and superdiagonal entries of the bidiagonalized
	//	  version of the U matrix
	//	  are stored in the vectors w and superDiag (temporarily).

	// Apply Householder transformations to U.
	// Householder transformations come in pairs.
	//   First, on the left, we map a portion of a column to zeros
	//   Second, on the right, we map a portion of a row to zeros
	int rowStep = U.m_num_cols;
	int diagStep = U.m_num_cols + 1;
	double *diagPtr = U.x;
	double *wPtr = w.x;
	double *superDiagPtr = superDiag.x;
	int colLengthLeft = U.m_num_rows;
	int rowLengthLeft = V.m_num_cols;
	while (true) {
		// Apply a Householder xform on left to zero part of a column
		this->svd_householder(diagPtr, colLengthLeft, rowLengthLeft, 1, rowStep, wPtr);

		if ( rowLengthLeft==2 ) {
			*superDiagPtr = *(diagPtr+rowStep);
			break;
		}
		// Apply a Householder xform on the right to zero part of a row
		this->svd_householder(diagPtr+rowStep, rowLengthLeft-1, colLengthLeft, rowStep, 1, superDiagPtr);

		rowLengthLeft--;
		colLengthLeft--;
		diagPtr += diagStep;
		wPtr++;
		superDiagPtr++;
	}

	int extra = 0;
	diagPtr += diagStep;
	wPtr++;
	if ( colLengthLeft > 2 ) {
		extra = 1;
		// Do one last Householder transformation when the matrix is not square
		colLengthLeft--;
		this->svd_householder(diagPtr, colLengthLeft, 1, 1, 0, wPtr);
	}
	else {
		*wPtr = *diagPtr;
	}

	// Form U and V from the Householder transformations
	V.expand_householders(V.m_num_cols - 2, 1, U.x + U.m_num_rows, U.m_num_rows, 1);
	U.expand_householders(V.m_num_cols - 1 + extra, 0, U.x, 1, U.m_num_rows);
}

/*
 * Helper routine for CalcBidiagonal.  Performs a series of Householder
 * transformations on a matrix. Stores results compactly into the matrix:
 *   The Householder vector u (normalized) is stored into the first row/column
 * being transformed. The leading term of that row (= plus/minus its magnitude
 * is returned separately into "retFirstEntry"
 */
void MatrixRmn::svd_householder(double *basePt, int colLength, int numCols, int colStride, int rowStride, double *retFirstEntry) {
	// Calc norm of vector u
	double* cPtr = basePt;
	double norm = 0.0;
	long i;
	for ( i=colLength; i>0 ; i-- ) {
		norm += (*cPtr)*(*cPtr);
		cPtr += colStride;
	}
	norm = sqrt(norm);					// Norm of vector to reflect to axis  e_1

	// Handle sign issues
	double imageVal;					// Choose sign to maximize distance
	if ( (*basePt) < 0.0 ) {
		imageVal = norm;
		norm = 2.0*norm*(norm-(*basePt));
	}
	else {
		imageVal = -norm;
		norm = 2.0*norm*(norm+(*basePt));
	}
	norm = sqrt(norm);					// Norm is norm of reflection vector

	if ( norm==0.0 ) {			// If the vector being transformed is equal to zero
		// Force to zero in case of roundoff errors
		cPtr = basePt;
		for ( i=colLength; i>0; i-- ) {
			*cPtr = 0.0;
			cPtr += colStride;
		}
		*retFirstEntry = 0.0;
		return;
	}

	*retFirstEntry = imageVal;

	// Set up the normalized Householder vector
	*basePt -= imageVal;					// First component changes. Rest stay the same.
	// Normalize the vector
	norm = 1.0/norm;					// Now it is the inverse norm
	cPtr = basePt;
	for ( i=colLength; i>0 ; i-- ) {
		*cPtr *= norm;
		cPtr += colStride;
	}

	// Transform the rest of the U matrix with the Householder transformation
	double *rPtr = basePt;
	for ( long j=numCols-1; j>0; j-- ) {
		rPtr += rowStride;
		// Calc dot product with Householder transformation vector
		double dotP = dot_array( colLength, basePt, colStride, rPtr, colStride );
		// Transform with I - 2*dotP*(Householder vector)
		addArrayScale( colLength, basePt, colStride, rPtr, colStride, -2.0*dotP );
	}
}

/*
 * Expand Householders
 *  The matrix will be square.
 *   numXforms: number of Householder transformations to concatenate
 *		Each Householder transformation is represented by a unit vector
 *		Each successive Householder transformation starts one position later
 *		and has one more implied leading zero
 *   basePt: beginning of the first Householder transform
 *   colStride, rowStride: Householder xforms are stored in "columns"
 *   numZerosSkipped is the number of implicit zeros on the front each
 *		Householder transformation vector (only values supported are 0 and 1).
 */
void MatrixRmn::expand_householders(int numXforms, int numZerosSkipped, double *basePt, int colStride, int rowStride) {
	long numToTransform = m_num_cols-numXforms+1-numZerosSkipped;
	assert( numToTransform>0 );

	if ( numXforms==0 ) {
		setIdentity();
		return;
	}

	// Handle the first one separately as a special case,
	// "this" matrix will be treated to simulate being preloaded with the identity
	long hDiagStride = rowStride+colStride;
	double* hBase = basePt + hDiagStride*(numXforms-1);	// Pointer to the last Householder vector
	double* hDiagPtr = hBase + colStride*(numToTransform-1);		// Pointer to last entry in that vector
	long i;
	double* diagPtr = x+m_num_cols*m_num_rows-1;					// Last entry in matrix (points to diagonal entry)
	double* colPtr = diagPtr-(numToTransform-1);			// Pointer to column in matrix
	for ( i=numToTransform; i>0; i-- ) {
		this->copy_array_scale( numToTransform, hBase, colStride, colPtr, 1, -2.0*(*hDiagPtr) );
		*diagPtr += 1.0;						// Add back in 1 to the diagonal entry (since xforming the identity)
		diagPtr -= (m_num_rows+1);					// Next diagonal entry in this matrix
		colPtr -= m_num_rows;						// Next column in this matrix
		hDiagPtr -= colStride;
	}

	// Now handle the general case
	// A row of zeros must be in effect added to the top of each old column (in each loop)
	double* colLastPtr = x + this->m_size - numToTransform - 1;
	for ( i = numXforms-1; i>0; i-- ) {
		numToTransform++;							// Number of non-trivial applications of this Householder transformation
		hBase -= hDiagStride;						// Pointer to the beginning of the Householder transformation
		colPtr = colLastPtr;
		for ( long j = numToTransform-1; j>0; j-- ) {
			// Get dot product
			double dotProd2N = -2.0*dot_array( numToTransform-1, hBase+colStride, colStride, colPtr+1, 1 );
			*colPtr = dotProd2N*(*hBase);			// Adding onto zero at initial point
			this->addArrayScale( numToTransform-1, hBase+colStride, colStride, colPtr+1, 1, dotProd2N );
			colPtr -= m_num_rows;
		}
		// Do last one as a special case (may overwrite the Householder vector)
		this->copy_array_scale( numToTransform, hBase, colStride, colPtr, 1, -2.0*(*hBase) );
		*colPtr += 1.0;				// Add back one one as identity
		// Done with this Householder transformation
		colLastPtr --;
	}

	if ( numZerosSkipped!=0 ) {
		assert( numZerosSkipped==1 );
		// Fill first row and column with identity (More generally: first numZerosSkipped many rows and columns)
		double* d = x;
		*d = 1;
		double* d2 = d;
		for ( i=m_num_rows-1; i>0; i-- ) {
			*(++d) = 0;
			*(d2+=m_num_rows) = 0;
		}
	}
}

/*
 * ConvertBidiagToDiagonal
 *  Do the iterative transformation from bidiagonal form to diagonal form using
 *  Givens transformation.  (Golub-Reinsch) U and V are square.  Size of U less
 *  than or equal to that of U.
 */
void MatrixRmn::convert_bidiag_to_diag(MatrixRmn& U, MatrixRmn& V, VectorRn& w, VectorRn& superDiag) {
	// These two index into the last bidiagonal block  (last in the matrix, it will be first one handled.
	int lastBidiagIdx = V.m_num_rows - 1;
	int firstBidiagIdx = 0;
	double eps = 1.0e-15 * ((w.maxAbs() > superDiag.maxAbs()) ? w.maxAbs() : superDiag.maxAbs());

	while ( true ) {
		bool workLeft = this->update_bidiag_indices(&firstBidiagIdx, &lastBidiagIdx, w, superDiag, eps);
		if ( !workLeft ) {
			break;
		}

		// Get ready for first Givens rotation
		// Push non-zero to M[2,1] with Givens transformation
		double* wPtr = w.x+firstBidiagIdx;
		double* sdPtr = superDiag.x+firstBidiagIdx;
		double extraOffDiag=0.0;
		if ( (*wPtr)==0.0 ) {
			this->clear_row_with_diagonal_zero(firstBidiagIdx, lastBidiagIdx, U, wPtr, sdPtr, eps);
			if ( firstBidiagIdx > 0 ) {
				if ( fabs(*(--sdPtr)) <= eps ) {
					*sdPtr = 0.0;
				}
				else {
					this->clear_column_with_diagonal_zero(firstBidiagIdx, V, wPtr, sdPtr, eps);
				}
			}
			continue;
		}

		// Estimate an eigenvalue from bottom four entries of M
		// This gives a lambda value which will shift the Givens rotations
		// Last four entries of M^T * M are  ( ( A, B ), ( B, C ) ).
		double A = (firstBidiagIdx<lastBidiagIdx-1) ? superDiag[lastBidiagIdx-2]*superDiag[lastBidiagIdx-2]: 0.0;
		double BSq = w[lastBidiagIdx-1]*w[lastBidiagIdx-1];
		A += BSq;										// The "A" entry of M^T * M
		double C = superDiag[lastBidiagIdx-1]*superDiag[lastBidiagIdx-1];
		BSq *= C;										// The squared "B" entry
		C += w[lastBidiagIdx]*w[lastBidiagIdx];					// The "C" entry
		double lambda;									// lambda will hold the estimated eigenvalue
		lambda = sqrt( ((A-C)*0.5)*((A-C)*0.5) + BSq );		// Use the lambda value that is closest to C.
		if ( A > C ) {
			lambda = -lambda;
		}
		lambda += (A+C)*0.5;						// Now lambda equals the estimate for the last eigenvalue
		double t11 = w[firstBidiagIdx]*w[firstBidiagIdx];
		double t12 = w[firstBidiagIdx]*superDiag[firstBidiagIdx];

		double c, s;
        this->calc_givens_values(t11-lambda, t12, &c, &s);
		this->apply_givens_cbtd(c, s, wPtr, sdPtr, &extraOffDiag, wPtr+1);
        V.postApplyGivens(c, -s, firstBidiagIdx, firstBidiagIdx + 1);
		int i;
		for ( i = firstBidiagIdx; i < lastBidiagIdx - 1; i++ ) {
			// Push non-zero from M[i+1,i] to M[i,i+2]
			this->calc_givens_values(*wPtr, extraOffDiag, &c, &s);
			this->apply_givens_cbtd(c, s, wPtr, sdPtr, &extraOffDiag, extraOffDiag, wPtr+1, sdPtr+1);
			U.postApplyGivens(c, -s, i, i + 1);
			// Push non-zero from M[i,i+2] to M[1+2,i+1]
            this->calc_givens_values( *sdPtr, extraOffDiag, &c, &s );
			this->apply_givens_cbtd(c, s, sdPtr, wPtr+1, &extraOffDiag, extraOffDiag, sdPtr+1, wPtr+2);
			V.postApplyGivens(c, -s, i + 1, i + 2);
			wPtr++;
			sdPtr++;
		}
		// Push non-zero value from M[i+1,i] to M[i,i+1] for i==lastBidiagIdx-1
		this->calc_givens_values(*wPtr, extraOffDiag, &c, &s);
		this->apply_givens_cbtd(c, s, wPtr, &extraOffDiag, sdPtr, wPtr+1);
		U.postApplyGivens(c, -s, i, i + 1);
	}
}

/*
 * This is called when there is a zero diagonal entry, with a non-zero
 * superdiagonal entry on the same row.  We use Givens rotations to "chase" the
 * non-zero entry across the row; when it reaches the last column, it is finally
 * zeroed away. wPtr points to the zero entry on the diagonal.  sdPtr points to
 * the non-zero superdiagonal entry on the same row.
 */
void MatrixRmn::clear_row_with_diagonal_zero(int firstBidiagIdx, int lastBidiagIdx, MatrixRmn& U, double *wPtr, double *sdPtr, double eps) {
	double curSd = *sdPtr;		// Value being chased across the row
	*sdPtr = 0.0;
	long i = firstBidiagIdx + 1;
	while (true) {
		// Rotate row i and row firstBidiagIdx (Givens rotation)
		double c, s;
        this->calc_givens_values( *(++wPtr), curSd, &c, &s );
		U.postApplyGivens(c, -s, i, firstBidiagIdx);
		*wPtr = c*(*wPtr) - s*curSd;
		if ( i == lastBidiagIdx ) {
			break;
		}
		curSd = s*(*(++sdPtr));		// New value pops up one column over to the right
		*sdPtr = c*(*sdPtr);
		i++;
	}
}

/*
 * This is called when there is a zero diagonal entry, with a non-zero
 * superdiagonal entry in the same column.  We use Givens rotations to "chase"
 * the non-zero entry up the column; when it reaches the last column, it is
 * finally zeroed away. wPtr points to the zero entry on the diagonal.  sdPtr
 * points to the non-zero superdiagonal entry in the same column.
 */
void MatrixRmn::clear_column_with_diagonal_zero(int endIdx, MatrixRmn& V, double *wPtr, double *sdPtr, double eps) {
	double curSd = *sdPtr;		// Value being chased up the column
	*sdPtr = 0.0;
	long i = endIdx-1;
	while ( true ) {
		double c, s;
        this->calc_givens_values( *(--wPtr), curSd, &c, &s );
		V.postApplyGivens(c, -s, i, endIdx);
		*wPtr = c*(*wPtr) - s*curSd;
		if ( i==0 ) {
			break;
		}
		curSd = s*(*(--sdPtr));		// New value pops up one row above
		if ( fabs(curSd) <= eps ) {
			break;
		}
		*sdPtr = c*(*sdPtr);
		i--;
	}
}

// Matrix A is  ( ( a c ) ( b d ) ), i.e., given in column order.
// Mult's G[c,s]  times  A, replaces A.
void MatrixRmn::apply_givens_cbtd(double cosine, double sine, double *a, double *b, double *c, double *d) {
	double temp = *a;
	*a = cosine*(*a) - sine*(*b);
	*b = sine*temp + cosine*(*b);
	temp = *c;
	*c = cosine*(*c) - sine*(*d);
	*d = sine*temp + cosine*(*d);
}

// Now matrix A given in row order, A = ( ( a b c ) ( d e f ) ).
// Return G[c,s] * A, replace A.  d becomes zero, no need to return.
//  Also, it is certain the old *c value is taken to be zero!
void MatrixRmn::apply_givens_cbtd(double cosine, double sine, double *a, double *b, double *c, double  d, double *e, double *f) {
	*a = cosine*(*a) - sine*d;
	double temp = *b;
	*b = cosine*(*b) - sine*(*e);
	*e = sine*temp + cosine*(*e);
	*c =  -sine*(*f);
	*f =  cosine*(*f);
}

// Helper routine for SVD conversion from bidiagonal to diagonal
bool MatrixRmn::update_bidiag_indices(int *firstBidiagIdx, int *lastBidiagIdx, VectorRn& w, VectorRn& superDiag, double eps) {
	long lastIdx = *lastBidiagIdx;
	double* sdPtr = superDiag.getPtr( lastIdx-1 );		// Entry above the last diagonal entry
	//while ( NearZero(*sdPtr, eps) ) {
	while ( fabs(*sdPtr) <= eps ) {
		*(sdPtr--) = 0.0;
		lastIdx--;
		if ( lastIdx == 0 ) {
			return false;
		}
	}
	*lastBidiagIdx = lastIdx;
	long firstIdx = lastIdx-1;
	double* wPtr = w.getPtr( firstIdx );
	while ( firstIdx > 0 ) {
		//if ( NearZero( *wPtr, eps ) ) {			// If this diagonal entry (near) zero
		if ( fabs(*wPtr) <= eps ) {			// If this diagonal entry (near) zero
			*wPtr = 0.0;
			break;
		}
		//if ( NearZero(*(--sdPtr), eps) ) {		// If the entry above the diagonal entry is (near) zero
		if ( fabs(*(--sdPtr)) <= eps ) {		// If the entry above the diagonal entry is (near) zero
			*sdPtr = 0.0;
			break;
		}
		wPtr--;
		firstIdx--;
	}
	*firstBidiagIdx = firstIdx;
	return true;
}

// Helper routine: calculate dot product
double MatrixRmn::dot_array(int length, double *ptrA, int strideA, double *ptrB, int strideB) {
    double result = 0;
    for ( ; length>0 ; length-- ) {
        result += (*ptrA)*(*ptrB);
        ptrA += strideA;
        ptrB += strideB;
    }
    return result;
}

int MatrixRmn::get_diagonal_length(void) {
    return ((this->m_num_rows < this->m_num_cols) ? this->m_num_rows : this->m_num_cols);
}

// Helper routine: copies and scales an array
void MatrixRmn::copy_array_scale(int length, double *from, int fromStride, double *to, int toStride, double scale) {
    for ( ; length>0; length-- ) {
        *to = (*from)*scale;
        from += fromStride;
        to += toStride;
    }
}

// The matrix A is loaded, in into "this" matrix, based at (0,0).
//  The size of "this" matrix must be large enough to accomodate A.
//  The rest of "this" matrix is left unchanged.  It is not filled with zeroes!
void MatrixRmn::load_as_submatrix(MatrixRmn& A) {
    assert( A.m_num_rows<=m_num_rows && A.m_num_cols<=m_num_cols );
    int extraColStep = m_num_rows - A.m_num_rows;
    double *to = x;
    double *from = A.x;
    for ( long i=A.m_num_cols; i>0; i-- ) {            // Copy columns of A, one per time thru loop
        for ( long j=A.m_num_rows; j>0; j-- ) {        // Copy all elements of this column of A
            *(to++) = *(from++);
        }
        to += extraColStep;
    }
}

// The matrix A is loaded, in transposed order into "this" matrix, based at (0,0).
//  The size of "this" matrix must be large enough to accomodate A.
//  The rest of "this" matrix is left unchanged.  It is not filled with zeroes!
void MatrixRmn::load_as_submatrix_transpose(MatrixRmn& A) {
    assert( A.m_num_rows<=m_num_cols && A.m_num_cols<=m_num_rows );
    double* rowPtr = x;
    double* from = A.x;
    for ( long i=A.m_num_cols; i>0; i-- ) {                // Copy columns of A, once per loop
        double* to = rowPtr;
        for ( long j=A.m_num_rows; j>0; j-- ) {            // Loop copying values from the column of A
            *to = *(from++);
            to += m_num_rows;
        }
        rowPtr ++;
    }
}

// Calculate the c=cosine and s=sine values for a Givens transformation.
void MatrixRmn::calc_givens_values(double a, double b, double *c, double *s) {
    double denomInv = sqrt(a*a + b*b);
    if ( denomInv==0.0 ) {
        *c = 1.0;
        *s = 0.0;
    }
    else {
        denomInv = 1.0/denomInv;
        *c = a*denomInv;
        *s = -b*denomInv;
    }
}