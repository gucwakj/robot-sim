#include <iostream>
#include "jacobian.h"
#include "node.h"

Jacobian::Jacobian(Tree *tree, VectorR3 *target_pos, MatrixR33 *target_rot) {
	this->tree = tree;
	this->target_pos = target_pos;
    this->target_rot = target_rot;

	this->m_num_effect = this->tree->getNumEffector();
	this->m_num_joint = this->tree->getNumJoint();
	this->m_num_row = 6 * this->m_num_effect;		        // 6dof
	this->m_num_col = this->m_num_joint;
	this->m_lambda_default = 1.1;							//  DLS: default damping
	this->m_pi_factor = 0.01;								//   PI: threshhold for eigenvalues = 0
	this->m_base_max_dist = 0.4;							// SDLS: ?????
	this->m_max_angle = new double[NUM_MODES];				// array of max angle jumps for each time step
	this->m_max_angle[JACOB_UNDEFINED] = 0;
	this->m_max_angle[JACOB_TRANSPOSE] = 30.0*M_PI/180;
	this->m_max_angle[JACOB_PSEUDOINVERSE] = 5.0*M_PI/180;
	this->m_max_angle[JACOB_DLS] = 45.0*M_PI/180;
	this->m_max_angle[JACOB_DLS_SVD] = 45.0*M_PI/180;
	this->m_max_angle[JACOB_SDLS] = 45.0*M_PI/180;

	this->J.setSize(this->m_num_row, this->m_num_col);

    this->U.setSize(this->m_num_row, this->m_num_row);
    this->w.setLength( (this->m_num_row < this->m_num_col) ? this->m_num_row : this->m_num_col );
	this->V.setSize(this->m_num_col, this->m_num_col);

    this->Jnorms.setSize(this->m_num_effect, this->m_num_col);  // Holds the norms of the active J matrix
    this->dPreTheta.setLength(this->m_num_col);     // delta theta for single eigenvalue
    this->dS.setLength(this->m_num_row);			// (target positions) - (end effector positions)
    this->dSclamp.setLength(this->m_num_effect);    // clamp magnitude of dT
    this->dT.setLength(this->m_num_row);			// linearized change in end effector positions based on dTheta
    this->dTheta.setLength(this->m_num_col);        // changes in joint angles

	this->reset();
}

Jacobian::~Jacobian(void) {
	delete this->m_max_angle;
}

void Jacobian::computeJacobian(void) {
	int i = 0, j = 0;
	VectorR3 pos, rot, rot2;
    MatrixR33 R;
	Node *n = this->tree->getRoot();

	while ( n ) {
		if ( n->isEffector() ) {
			i = n->getEffectorNum();

            pos = this->target_pos[i] - n->getS();      // position
            this->dS.setTriplePosition(i, pos);
            R = n->getR();
            rot = 0.5*(R.getColumn1()*this->target_rot[i].getColumn1() + R.getColumn2()*this->target_rot[i].getColumn2() + R.getColumn3()*this->target_rot[i].getColumn3());
            //rot = VectorR3(this->target_rot[i].psi, this->target_rot[i].theta, this->target_rot[i].phi) - VectorR3(R.psi, R.theta, R.phi);
            this->dS.setTripleRotation(i, rot);

			Node *m = this->tree->getParent(n);
			while ( m ) {
				j = m->getJointNum();
				assert( 0 <=i && i < this->m_num_effect && 0 <= j && j < this->m_num_joint );

				if ( m->isFrozen() ) {
                    this->J.setTriplePosition(i, j, VectorR3(0, 0, 0));
                    this->J.setTripleRotation(i, j, VectorR3(0, 0, 0));
				}
				else {
					pos = m->getS();					    // joint pos
					if ( this->m_j_type == J_END )
						pos -= n->getS();				    // -(end effector pos - joint pos)
					else
						pos -= this->target_pos[i];	        // -(target pos - joint pos)
                    pos *= m->getW();					    // cross product with joint rotation axis
                    this->J.setTriplePosition(i, j, pos);
                    rot = m->getW();
                    this->J.setTripleRotation(i, j, rot);
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
			temp = this->target_pos[i] - n->getS();

            changedDist = temp.norm() - this->dS.norm(i);
			if ( changedDist > 0.0 )
				this->dSclamp[i] = this->m_base_max_dist + changedDist;
			else
				this->dSclamp[i] = this->m_base_max_dist;
		}
		n = this->tree->getSuccessor(n);
	}
}

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
    this->dSclamp.setValue(HUGE_VAL);
}

// Find the delta theta values using inverse Jacobian method
void Jacobian::calc_delta_thetas_transpose(void) {
	this->J.MultiplyTranspose(this->dS, this->dTheta);

	// Scale back the dTheta values greedily
	this->J.Multiply(this->dTheta, this->dT);								// dT = J * dTheta

    double alpha = this->dS.dot(this->dT) / (this->dT.norm()*this->dT.norm());
	double beta = this->m_max_angle[JACOB_TRANSPOSE] / dTheta.maxAbs();
	assert( alpha > 0.0 );

	this->dTheta *= ((alpha < beta) ? alpha : beta);
}

// Calculate response vector dTheta that is the DLS solution.
//	Delta target values are the dS values
//  We multiply by Moore-Penrose pseudo-inverse of the J matrix
void Jacobian::calc_delta_thetas_pseudoinverse(void) {
	double alpha = 0, dotProdCol = 0;

	this->J.computeSVD(U, w, V);			// Compute SVD
    assert(this->J.DebugCheckSVD(U, w, V));		// Debugging check

	int diagLength = this->w.getLength();
	double *wPtr = this->w.getPtr();
	this->dTheta.setValue(0);

	for ( int i = 0; i < diagLength; i++ ) {
		dotProdCol = this->U.DotProductColumn(this->dS, i);		// Dot product with i-th column of U
		alpha = *(wPtr++);
		if ( fabs(alpha) > (this->m_pi_factor * this->w.maxAbs()) ) {
			alpha = 1.0/alpha;
			MatrixRmn::AddArrayScale(this->V.getNumRows(), this->V.getColumnPtr(i), 1, this->dTheta.getPtr(), 1, dotProdCol*alpha);
		}
	}

	// Scale back to not exceed maximum angle changes
	scale_back_angle(JACOB_PSEUDOINVERSE);
}

void Jacobian::calc_delta_thetas_dls(void) {
	MatrixRmn::MultiplyTranspose(J, J, U);		// U = J * (J^T)
	this->U.addToDiagonal(this->m_lambda*this->m_lambda);

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

	this->J.computeSVD(U, w, V);			// Compute SVD
    assert(this->J.DebugCheckSVD(U, w, V));		// Debugging check

	//int diagLength = this->w.GetLength();
	double *wPtr = this->w.getPtr();
    this->dTheta.setValue(0);

	for ( int i = 0; i < this->w.getLength(); i++ ) {
		dotProdCol = this->U.DotProductColumn(this->dS, i);		// Dot product with i-th column of U
		alpha = *(wPtr++);
		alpha = alpha / (alpha * alpha + this->m_lambda * this->m_lambda);
		MatrixRmn::AddArrayScale(this->V.getNumRows(), this->V.getColumnPtr(i), 1, this->dTheta.getPtr(), 1, dotProdCol*alpha);
	}

	// Scale back to not exceed maximum angle changes
	scale_back_angle(JACOB_DLS);
}

void Jacobian::calc_delta_thetas_sdls(void) {
	int i, j, k;
	double alpha, wiInv, accum, accumSq, N, temp, tempSq, M, gamma;

	this->J.computeSVD(this->U, this->w, this->V);				// Compute SVD
	assert(this->J.DebugCheckSVD(this->U, this->w, this->V));	// Debugging check

	// Calculate response vector dTheta that is the SDLS solution.
    //	Delta target values are the dS values
	int num_rows = this->J.getNumRows();
	int num_cols = this->J.getNumColumns();
	int num_effectors = this->tree->getNumEffector();		// Equals the number of rows of J divided by three
    this->dTheta.setValue(0);

	// Calculate the norms of the 3-vectors in the Jacobian
	const double *jx = J.getPtr();
	double *jnx = Jnorms.getPtr();
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
		const double *dTx = this->dT.getPtr();
		const double *ux = this->U.getColumnPtr(i);
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
		double *vx = this->V.getColumnPtr(i);
		jnx = this->Jnorms.getPtr();
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
        this->dPreTheta.set(this->V.getColumnPtr(i), alpha*wiInv);
		// Now rescale the dTheta values
        this->dTheta.add(this->dPreTheta, gamma/(gamma + this->dPreTheta.maxAbs()));
	}
	scale_back_angle(JACOB_SDLS, 1);		// Scale back to not exceed maximum angle changes
}

void Jacobian::zero_delta_thetas(void) {
    this->dTheta.setValue(0);
}

void Jacobian::calc_dT_clamped_from_dS(void) {
    for ( int i = 0, j = 0; i < this->dS.getLength(); i+=6, j++ ) {
        if ( (this->dS.norm(i)*this->dS.norm(i)) > (this->dSclamp[j]*this->dSclamp[j]) )
            this->dT.setTriplePosition(i, this->dSclamp[j]*VectorR3(this->dS[i+0], this->dS[i+1], this->dS[i+2])/this->dS.norm(i));
        else
            this->dT.setTriplePosition(i, VectorR3(this->dS[i+0], this->dS[i+1], this->dS[i+2]));

        if ( (this->dS.norm(i+3)*this->dS.norm(i+3)) > (this->dSclamp[j]*this->dSclamp[j]) )
            this->dT.setTripleRotation(i, this->dSclamp[j]*VectorR3(this->dS[i+3], this->dS[i+4], this->dS[i+5])/this->dS.norm(i+3));
        else
            this->dT.setTripleRotation(i, VectorR3(this->dS[i+3], this->dS[i+4], this->dS[i+5]));
	}
}

void Jacobian::scale_back_angle(int j_mode, int method) {
	double maxChange = this->dTheta.maxAbs();
	if ( maxChange > this->m_max_angle[j_mode] && method )
		this->dTheta *= (this->m_max_angle[j_mode] / (this->m_max_angle[j_mode] + maxChange));
	else if ( maxChange > this->m_max_angle[j_mode] && !method )
		this->dTheta *= (this->m_max_angle[j_mode] / maxChange);
}
