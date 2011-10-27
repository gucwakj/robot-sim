#include <iostream>
#include "jacobian.h"
#include "node.h"

Jacobian::Jacobian(Tree *tree, VectorR3 *target_pos, MatrixR33 *target_rot) {
	this->tree = tree;
	this->target_pos = target_pos;
    this->target_rot = target_rot;

	this->m_num_effect = this->tree->getNumEffector();
	this->m_num_joint = this->tree->getNumJoint();
	//this->m_num_row = 3 * this->m_num_effect;		// 3dof
	this->m_num_row = 6 * this->m_num_effect;		// 6dof
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

	this->U.SetSize(this->m_num_row, this->m_num_row);
	this->V.SetSize(this->m_num_col, this->m_num_col);
	this->w.SetLength( (this->m_num_row < this->m_num_col) ? this->m_num_row : this->m_num_col );

    this->dTheta.SetLength(this->m_num_col);		// changes in joint angles
    this->dS.SetLength(3 * this->m_num_effect);         // (target positions) - (end effector positions)
    this->dT.SetLength(3 * this->m_num_effect);         // linearized change in end effector positions based on dTheta
    //this->dS.SetLength(this->m_num_row);			// (target positions) - (end effector positions)
    //this->dT.SetLength(this->m_num_row);			// linearized change in end effector positions based on dTheta
	//this->dQ.SetLength(3 * this->m_num_effect);			//
	//this->dR.SetLength(3 * this->m_num_effect);			// linearized change in end effector orientations based on dTheta

	this->dSclamp.SetLength(this->m_num_effect);
	//this->errorArray.SetLength(this->m_num_effect);
	this->Jnorms.SetSize(this->m_num_effect, this->m_num_col);	// Holds the norms of the active J matrix
	this->dPreTheta.SetLength(this->m_num_col);

	this->reset();
}

Jacobian::~Jacobian(void) {
	delete this->m_max_angle;
}

void Jacobian::computeJacobian(void) {
	int i = 0, j = 0;
	VectorR3 temp;
	Node *n = this->tree->getRoot();

	while ( n ) {
		if ( n->isEffector() ) {
			i = n->getEffectorNum();
			// position
			temp = this->target_pos[i] - n->getS();
			this->dS.SetTriple(i, temp);
			// orientation
			//temp = 0.5 * ();
			//this->dQ.SetTriple(i, temp);

			Node *m = this->tree->getParent(n);
			while ( m ) {
				j = m->getJointNum();
				assert( 0 <=i && i < this->m_num_effect && 0 <= j && j < this->m_num_joint );

				if ( m->isFrozen() ) {
					//this->J.SetTriple(i, j, VectorR3(0, 0, 0));			// position
					this->J.SetHextuple(i, j, VectorR3(0, 0, 0), VectorR3(0, 0, 0));
				}
				else {
					temp = m->getS();					// joint pos
					if ( this->m_j_type == J_END )
						temp -= n->getS();				// -(end effector pos - joint pos)
					else
						temp -= this->target_pos[i];	// -(target pos - joint pos)
					temp *= m->getW();					// cross product with joint rotation axis
					//this->J.SetTriple(i, j, temp);		// set position
					this->J.SetHextuple(i, j, temp, m->getW());
				}
				m = this->tree->getParent(m);
			}
		}
		n = this->tree->getSuccessor(n);
	}
	//cout << "J Rows: " << this->J.GetNumRows() << endl;
	//cout << "J Cols: " << this->J.GetNumColumns() << endl;
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
			temp = this->target_pos[i] - n->getS();

            changedDist = temp.Norm() - this->dS.norm(i);
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
			temp = this->target_pos[i] - n->getS();
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

    double alpha = Dot(this->dS, this->dT) / this->dT.normSq();
    //double alpha2 = this->dS.dot(this->dT) / this->dT.NormSq();   // will be implemented when dS length
    //cout << "alpha: " << alpha << "\talpha2: " << alpha2 << endl; // issue is sorted out
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
	int i, j, k;
	double alpha, wiInv, accum, accumSq, N, temp, tempSq, M, gamma;

	this->J.ComputeSVD(this->U, this->w, this->V);				// Compute SVD
	assert(this->J.DebugCheckSVD(this->U, this->w, this->V));	// Debugging check

	// Calculate response vector dTheta that is the SDLS solution.
	//	Delta target values are the dS values
	int num_rows = this->J.GetNumRows();
	int num_cols = this->J.GetNumColumns();
	int num_effectors = this->tree->getNumEffector();		// Equals the number of rows of J divided by three
	this->dTheta.SetZero();

	// Calculate the norms of the 3-vectors in the Jacobian
	const double *jx = J.GetPtr();
	double *jnx = Jnorms.GetPtr();
	for ( i = num_cols*num_effectors; i > 0; i-- ) {
		accum = *(jx++);
		accumSq = accum*accum;
		accum = *(jx++);
		accumSq += accum*accum;
		accum = *(jx++);
		accumSq += accum*accum;
		*(jnx++) = sqrt(accumSq);
	}

	this->calc_dT_clamped_from_dS();		// Clamp the dS values

	// Loop over each singular vector
	for ( i = 0; i < num_rows; i++ ) {
		if ( fabs(this->w[i]) <= 1.0e-10 ) { continue; }
		wiInv = 1.0 / this->w[i];

		// Calculate N
		N = 0;					// N is the quasi-1-norm of the i-th column of U
		alpha = 0;				// alpha is the dot product of dT and the i-th column of U
		const double *dTx = this->dT.GetPtr();
		const double *ux = this->U.GetColumnPtr(i);
		for ( j = num_effectors; j > 0; j-- ) {
			alpha += (*ux)*(*(dTx++));
			temp = *(ux++);
			tempSq = temp*temp;
			alpha += (*ux)*(*(dTx++));
			temp = *(ux++);
			tempSq += temp*temp;
			alpha += (*ux)*(*(dTx++));
			temp = *(ux++);
			tempSq += temp*temp;
			N += sqrt(tempSq);
		}

		// Calculate M
		M = 0;			// M is the quasi-1-norm of the response to angles changing according to the i-th column of V
		double *vx = this->V.GetColumnPtr(i);
		jnx = this->Jnorms.GetPtr();
		for ( j = num_cols; j > 0; j-- ) {
			double accum=0.0;
			for ( k = num_effectors; k > 0; k-- ) {
				accum += *(jnx++);
			}
			M += fabs((*(vx++)))*accum;
		}
		M *= fabs(wiInv);

		// Scale back maximum permissable joint angle
		gamma = this->m_max_angle[JACOB_SDLS];
		if ( N < M ) { gamma *= N/M; }

		// Calculate the dTheta from pure pseudoinverse considerations
		this->dPreTheta.LoadScaled( this->V.GetColumnPtr(i), alpha*wiInv);
		// Now rescale the dTheta values.
		this->dTheta.AddScaled(this->dPreTheta, gamma/(gamma + this->dPreTheta.MaxAbs()));
	}
	scale_back_angle(JACOB_SDLS, 1);		// Scale back to not exceed maximum angle changes
}

void Jacobian::zero_delta_thetas(void) {
	this->dTheta.SetZero();
}

void Jacobian::calc_dT_clamped_from_dS(void) {
    for ( int i = 0, j = 0; i < this->dS.GetLength(); i+=3, j++ ) {
        if ( this->dS.normSq(i) > (this->dSclamp[j]*this->dSclamp[j]) )
            this->dT.SetTriple(i, this->dSclamp[j]*VectorR3(this->dS[i], this->dS[i+1], this->dS[i+2])/this->dS.norm(i));
		else
            this->dT.SetTriple(i, VectorR3(this->dS[i], this->dS[i+1], this->dS[i+2]));
	}
}

void Jacobian::scale_back_angle(int j_mode, int method) {
	double maxChange = this->dTheta.MaxAbs();
	if ( maxChange > this->m_max_angle[j_mode] && method )
		this->dTheta *= (this->m_max_angle[j_mode] / (this->m_max_angle[j_mode] + maxChange));
	else if ( maxChange > this->m_max_angle[j_mode] && !method )
		this->dTheta *= (this->m_max_angle[j_mode] / maxChange);
}
