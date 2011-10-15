#include <iostream>
#include "jacobian.h"
#include "node.h"

Jacobian::Jacobian(Tree *tree, VectorR3 *target) {
	this->tree = tree;
	this->target = target;

	this->m_num_effect = this->tree->getNumEffector();
	this->m_num_joint = this->tree->getNumJoint();
	this->m_num_row = 3 * this->m_num_effect;
	this->m_num_col = this->m_num_joint;
	this->m_j_type = J_END;
	this->m_j_mode = JACOB_SDLS;
	this->m_dls_mode = TRADITIONAL;
	this->m_lambda_default = 1.1;							// DLS: Default damping
	this->m_pi_factor = 0.01;								// PI: threshhold for eigenvalues = 0
	this->m_base_max_dist = 0.4;							// SDLS: ?????
	this->m_max_angle = new double[NUM_MODES];				// array of max angle jumps for each time step
	this->m_max_angle[JACOB_UNDEFINED] = 0;
	this->m_max_angle[JACOB_TRANSPOSE] = 30.0*M_PI/180;
	this->m_max_angle[JACOB_PSEUDOINVERSE] = 5.0*M_PI/180;
	this->m_max_angle[JACOB_DLS] = 45.0*M_PI/180;
	this->m_max_angle[JACOB_DLS_SVD] = 45.0*M_PI/180;
	this->m_max_angle[JACOB_SDLS] = 45.0*M_PI/180;

	this->J.SetSize(this->m_num_row, this->m_num_col);
	this->J.SetZero();

	this->U.SetSize(this->m_num_row, this->m_num_row);				// The U matrix for SVD calculations
	this->V.SetSize(this->m_num_col, this->m_num_col);				// The V matrix for SVD calculations
	this->w.SetLength( (this->m_num_row < this->m_num_col) ? this->m_num_row : this->m_num_col );

	this->dS.SetLength(this->m_num_row);			// (Target positions) - (End effector positions)
	this->dTheta.SetLength(this->m_num_col);		// Changes in joint angles
	this->dT.SetLength(this->m_num_row);			// Linearized change in end effector positions based on dTheta

	// Used by the Selectively Damped Least Squares Method
	this->dSclamp.SetLength(this->m_num_effect);
	//this->errorArray.SetLength(this->m_num_effect);
	this->Jnorms.SetSize(this->m_num_effect, this->m_num_col);	// Holds the norms of the active J matrix
	this->dPreTheta.SetLength(this->m_num_col);

	this->reset();
}

Jacobian::~Jacobian(void) {
	delete this->m_max_angle;
	//delete this->tree;
	//delete this->target;
}

// Compute the deltaS vector, dS, (the error in end effector positions
// Compute the J and K matrices (the Jacobians)
void Jacobian::computeJacobian(void) {
	int i = 0, j = 0;
	VectorR3 temp;
	Node *n = this->tree->getRoot();

	while ( n ) {
		if ( n->isEffector() ) {
			i = n->getEffectorNum();
			temp = this->target[i] - n->getS();

			this->dS.SetTriple(i, temp);

			// Find all ancestors (they will usually all be joints)
			// Set the corresponding entries in the Jacobians J, K.
			Node *m = this->tree->getParent(n);
			while ( m ) {
				j = m->getJointNum();
				assert( 0 <=i && i < this->m_num_effect && 0 <= j && j < this->m_num_joint );

				if ( m->isFrozen() ) {
					//this->J.SetTriple(i, j, VectorR3::Zero);
					this->J.SetTriple(i, j, VectorR3(0, 0, 0));
				}
				else {
					temp = m->getS();				// joint pos
					if ( this->m_j_type == J_END )
						temp -= n->getS();			// -(end effector pos - joint pos)
					else {
						temp -= this->target[i];	// -(target pos - joint pos)
					}
					temp *= m->getW();				// cross product with joint rotation axis
					this->J.SetTriple(i, j, temp);
				}
				m = this->tree->getParent(m);
			}
		}
		n = this->tree->getSuccessor(n);
	}
}

void Jacobian::calcDeltaThetas(void) {
	switch (this->m_j_mode) {
		case JACOB_UNDEFINED:
			this->zero_delta_thetas();
			break;
		case JACOB_TRANSPOSE:
			this->calc_delta_thetas_transpose();
			break;
		case JACOB_PSEUDOINVERSE:
			this->calc_delta_thetas_pseudoinverse();
			break;
		case JACOB_DLS:
			this->calc_delta_thetas_dls();
			break;
		case JACOB_DLS_SVD:
			this->calc_delta_thetas_dls_with_svd();
			break;
		case JACOB_SDLS:
			this->calc_delta_thetas_sdls();
			break;
	}
}

// The delta theta values have been computed in dTheta array
// Apply the delta theta values to the joints
// Nothing is done about joint limits for now.
void Jacobian::updateThetas(void) {
	Node *n = this->tree->getRoot();

	while ( n ) {
		if ( n->isJoint() ) {
			n->updateTheta( this->dTheta[n->getJointNum()] );
		}
		n = this->tree->getSuccessor(n);
	}

	this->tree->compute();		// Update the positions and rotation axes of all joints/effectors
}

void Jacobian::updatedSClampValue(void) {
	int i = 0;
	double changedDist = 0.0;
	VectorR3 temp;
	Node *n = this->tree->getRoot();

	while ( n ) {
		if ( n->isEffector() ) {
			i = n->getEffectorNum();
			temp = this->target[i] - n->getS();

			changedDist = temp.Norm() - sqrt(dS[i]*dS[i] + dS[i+1]*dS[i+1] + dS[i+2]*dS[i+2]);
			if ( changedDist > 0.0 )
				this->dSclamp[i] = this->m_base_max_dist + changedDist;
			else
				this->dSclamp[i] = this->m_base_max_dist;
		}
		n = this->tree->getSuccessor(n);
	}
}

/*void Jacobian::updateErrorArray(void) {
	int i = 0;
	VectorR3 temp;
	Node *n = this->tree->getRoot();

	while ( n ) {
		if ( n->isEffector() ) {
			i = n->getEffectorNum();
			temp = this->target[i] - n->getS();
			this->errorArray[i] = temp.Norm();
		}
		n = this->tree->getSuccessor(n);
	}
}*/

void Jacobian::setCurrentType(int type) {	this->m_j_type = type;	}
void Jacobian::setCurrentMode(int mode) {	this->m_j_mode = mode;	}
void Jacobian::setCurrentDLSMode(int mode) {	this->m_dls_mode = mode;	}
void Jacobian::setDampingDLS(double lambda) {	this->m_lambda = lambda;	}

int Jacobian::getCurrentType(void) {	return this->m_j_type;	}
int Jacobian::getCurrentMode(void) {	return this->m_j_mode;	}
int Jacobian::getCurrentDLSMode(void) {	return this->m_dls_mode;}
double Jacobian::getDeltaTheta(int num) {	return this->dTheta[num]; }

void Jacobian::reset(void) {
	this->m_lambda = this->m_lambda_default;
	this->dSclamp.Fill(HUGE_VAL);
}

// Find the delta theta values using inverse Jacobian method
void Jacobian::calc_delta_thetas_transpose(void) {
	this->J.MultiplyTranspose(this->dS, this->dTheta);

	// Scale back the dTheta values greedily
	this->J.Multiply(this->dTheta, this->dT);								// dT = J * dTheta

	double alpha = Dot(this->dS, this->dT) / this->dT.NormSq();
	double beta = this->m_max_angle[JACOB_TRANSPOSE] / dTheta.MaxAbs();
	assert( alpha > 0.0 );

	this->dTheta *= ((alpha < beta) ? alpha : beta);
}

// Calculate response vector dTheta that is the DLS solution.
//	Delta target values are the dS values
//  We multiply by Moore-Penrose pseudo-inverse of the J matrix
void Jacobian::calc_delta_thetas_pseudoinverse(void) {
	double alpha = 0, dotProdCol = 0;

	this->J.ComputeSVD(U, w, V);			// Compute SVD
    assert(this->J.DebugCheckSVD(U, w, V));		// Debugging check

	int diagLength = this->w.GetLength();
	double *wPtr = this->w.GetPtr();
	this->dTheta.SetZero();

	for ( int i = 0; i < diagLength; i++ ) {
		dotProdCol = this->U.DotProductColumn(this->dS, i);		// Dot product with i-th column of U
		alpha = *(wPtr++);
		if ( fabs(alpha) > (this->m_pi_factor * this->w.MaxAbs()) ) {
			alpha = 1.0/alpha;
			MatrixRmn::AddArrayScale(this->V.GetNumRows(), this->V.GetColumnPtr(i), 1, this->dTheta.GetPtr(), 1, dotProdCol*alpha);
		}
	}

	// Scale back to not exceed maximum angle changes
	scale_back_angle(JACOB_PSEUDOINVERSE);
}

void Jacobian::calc_delta_thetas_dls(void) {
	MatrixRmn::MultiplyTranspose(J, J, U);		// U = J * (J^T)
	this->U.AddToDiagonal(this->m_lambda*this->m_lambda);

	if ( this->m_dls_mode == CLAMPED ) {		// DLS method with clamped error vector e.
		this->calc_dT_clamped_from_dS();
		VectorRn dTextra(3*this->m_num_effect);
		this->U.Solve( dT, &dTextra );
		this->J.MultiplyTranspose( dTextra, dTheta );
	}
	else {										// traditional DLS method
		this->U.Solve(this->dS, &(this->dT));
		this->J.MultiplyTranspose(this->dT, this->dTheta);
	}

	// Scale back to not exceed maximum angle changes
	scale_back_angle(JACOB_DLS);
}

// Calculate response vector dTheta that is the DLS solution.
//	Delta target values are the dS values
//  We multiply by DLS inverse of the J matrix
void Jacobian::calc_delta_thetas_dls_with_svd(void) {
	double alpha = 0, dotProdCol = 0;

	this->J.ComputeSVD(U, w, V);			// Compute SVD
    assert(this->J.DebugCheckSVD(U, w, V));		// Debugging check

	//int diagLength = this->w.GetLength();
	double *wPtr = this->w.GetPtr();
	this->dTheta.SetZero();

	for ( int i = 0; i < this->w.GetLength(); i++ ) {
		dotProdCol = this->U.DotProductColumn(this->dS, i);		// Dot product with i-th column of U
		alpha = *(wPtr++);
		alpha = alpha / (alpha * alpha + this->m_lambda * this->m_lambda);
		MatrixRmn::AddArrayScale(this->V.GetNumRows(), this->V.GetColumnPtr(i), 1, this->dTheta.GetPtr(), 1, dotProdCol*alpha);
	}

	// Scale back to not exceed maximum angle changes
	scale_back_angle(JACOB_DLS);
}

void Jacobian::calc_delta_thetas_sdls(void) {
	double alpha = 0, dotProdCol = 0;

	this->J.ComputeSVD(U, w, V);			// Compute SVD
	assert(this->J.DebugCheckSVD(U, w, V));		// Debugging check

	// Calculate response vector dTheta that is the SDLS solution.
	//	Delta target values are the dS values
	int nRows = this->J.GetNumRows();
	int numEndEffectors = this->tree->getNumEffector();		// Equals the number of rows of J divided by three
	int nCols = this->J.GetNumColumns();
	this->dTheta.SetZero();

	// Calculate the norms of the 3-vectors in the Jacobian
	int i;
	const double *jx = J.GetPtr();
	double *jnx = Jnorms.GetPtr();
	double accum, accumSq;
	for ( i = nCols*numEndEffectors; i > 0; i-- ) {
		accum = *(jx++);
		accumSq = accum*accum;
		accum = *(jx++);
		accumSq += accum*accum;
		accum = *(jx++);
		accumSq += accum*accum;
		*(jnx++) = sqrt(accumSq);
	}

	// Clamp the dS values
	this->calc_dT_clamped_from_dS();

	// Loop over each singular vector
	for ( i = 0; i < nRows; i++ ) {
		double wiInv = w[i];
		if ( fabs(wiInv) <= 1.0e-10 ) {
			continue;
		}
		wiInv = 1.0/wiInv;

		double N = 0.0;						// N is the quasi-1-norm of the i-th column of U
		alpha = 0.0;					// alpha is the dot product of dT and the i-th column of U

		const double *dTx = dT.GetPtr();
		const double *ux = U.GetColumnPtr(i);
		long j;
		double temp, temp2;
		for ( j = numEndEffectors; j > 0; j-- ) {
			alpha += (*ux)*(*(dTx++));
			temp = *(ux++);
			temp2 = temp*temp;
			alpha += (*ux)*(*(dTx++));
			temp = *(ux++);
			temp2 += temp*temp;
			alpha += (*ux)*(*(dTx++));
			temp = *(ux++);
			temp2 += temp*temp;
			N += sqrt(temp2);
		}

		// M is the quasi-1-norm of the response to angles changing according to the i-th column of V
		//		Then is multiplied by the wiInv value.
		double M = 0.0;
		double *vx = V.GetColumnPtr(i);
		jnx = Jnorms.GetPtr();
		for ( j=nCols; j>0; j-- ) {
			double accum=0.0;
			for ( long k=numEndEffectors; k>0; k-- ) {
				accum += *(jnx++);
			}
			M += fabs((*(vx++)))*accum;
		}
		M *= fabs(wiInv);

		double gamma = this->m_max_angle[JACOB_SDLS];
		if ( N<M ) {
			gamma *= N/M;				// Scale back maximum permissable joint angle
		}

		// Calculate the dTheta from pure pseudoinverse considerations
		double scale = alpha*wiInv;			// This times i-th column of V is the psuedoinverse response
		dPreTheta.LoadScaled( V.GetColumnPtr(i), scale );
		// Now rescale the dTheta values.
		double max = dPreTheta.MaxAbs();
		double rescale = (gamma)/(gamma+max);
		dTheta.AddScaled(dPreTheta,rescale);
	}

	// Scale back to not exceed maximum angle changes
	scale_back_angle(JACOB_SDLS, 1);
}

void Jacobian::zero_delta_thetas(void) {
	this->dTheta.SetZero();
}

void Jacobian::calc_dT_clamped_from_dS(void) {
	int j = 0, len = dS.GetLength();
	double factor = 0, normSq = 0;

	for ( int i = 0; i < len; i+=3, j++ ) {
		normSq = dS[i]*dS[i] + dS[i+1]*dS[i+1] + dS[i+2]*dS[i+2];
		if ( normSq > (this->dSclamp[j]*this->dSclamp[j]) ) {
			factor = this->dSclamp[j] / sqrt(normSq);
			dT[i] = dS[i]*factor;
			dT[i+1] = dS[i+1]*factor;
			dT[i+2] = dS[i+2]*factor;
		}
		else {
			dT[i] = dS[i];
			dT[i+1] = dS[i+1];
			dT[i+2] = dS[i+2];
		}
	}
}

void Jacobian::scale_back_angle(int j_mode, int method) {
	double maxChange = this->dTheta.MaxAbs();
	if ( maxChange > this->m_max_angle[j_mode] && method )
		this->dTheta *= (this->m_max_angle[j_mode] / (this->m_max_angle[j_mode] + maxChange));
	else if ( maxChange > this->m_max_angle[j_mode] && !method )
		this->dTheta *= (this->m_max_angle[j_mode] / maxChange);
}
