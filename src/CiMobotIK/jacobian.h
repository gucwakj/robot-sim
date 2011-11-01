#ifndef JACOBIAN_H_
#define JACOBIAN_H_

#include "matrixR33.h"
#include "matrixRmn.h"
#include "tree.h"

enum update_mode_e {
	JACOB_UNDEFINED,
	JACOB_TRANSPOSE,
	JACOB_PSEUDOINVERSE,
	JACOB_DLS,
	JACOB_DLS_SVD,
	JACOB_SDLS,
	NUM_MODES
};
enum jacobian_type_e {
	J_END,
	J_TARGET
};
enum dls_mode_e {
	CLAMPED,
	TRADITIONAL
};

class Jacobian {
	public:
		Jacobian(Tree *tree, VectorR3 *target_pos, MatrixR33 *target_rot);
		~Jacobian(void);

		void setCurrentMode(int mode);			// Type of updating mode for Jacobian
		void setCurrentType(int type);			// Jacobian type: END or TARGET
		void setCurrentDLSMode(int mode);		// CLAMPED or TRADITIONAL
		void setDampingDLS(double lambda);		// DLS damping

		int getCurrentMode(void);				// Type of updating mode for Jacobian
		int getCurrentType(void);				// Jacobian type: END or TARGET
		int getCurrentDLSMode(void);			// CLAMPED or TRADITIONAL
		double getDeltaTheta(int num);			// Current delta theta value of joint

        void computeJacobian(void);             // Compute Jacobian matrix
        void calcDeltaThetas(void);             // Calculate delta theta for each joint
        void updatedSClampValue(void);          // Update distance to target
        void updateThetas(void);                // Update theta values

		void reset(void);
	private:
		int m_num_effect;		// Number of end effectors
		int m_num_joint;		// Number of joints
		int m_num_row;			// Total number of rows the real J (= 3*number of end effectors for now)
		int m_num_col;			// Total number of columns in the real J (= number of joints for now)
		int m_j_type;			// Type of Jacobian matrix
		int m_j_mode;			// Type of update mode for Jacobian matrix
		int m_dls_mode;			// Type of update mode for DLS method
		double *m_max_angle;	// Cap on max. value of changes in angles in single update step
		double m_lambda;		// DLS: current lambda for DLS
		double m_lambda_default;// DLS: default lambda for DLS
		double m_base_max_dist;	// SDLS: ??????
		double m_pi_factor;		// PI: Threshold for treating eigenvalue as zero (fraction of largest eigenvalue)
		MatrixR33 *target_rot;  // target orientation for effectors
		MatrixRmn J;			// Jacobian matrix
		MatrixRmn Jnorms;		// Norms of 3-vectors in active Jacobian (SDLS only)
		MatrixRmn U;			// SVD (Singular Value Decomposition): J = U * Diag(w) * V^T
		MatrixRmn V;			// SVD (Singular Value Decomposition): J = U * Diag(w) * V^T
		Tree *tree;				// tree associated with this Jacobian matrix
		VectorR3 *target_pos;   // target positions for effectors
		VectorRn w;				// SVD (Singular Value Decomposition): J = U * Diag(w) * V^T
		VectorRn dS;			// delta s
		VectorRn dT;			// delta t		--  these are delta S values clamped to smaller magnitude
		VectorRn dSclamp;		// Value at which to clamp magnitude of dT
		VectorRn dTheta;		// delta theta
		VectorRn dPreTheta;		// delta theta for single eigenvalue  (SDLS only)

		void calc_delta_thetas_transpose(void);
		void calc_delta_thetas_pseudoinverse(void);
		void calc_delta_thetas_dls(void);
		void calc_delta_thetas_dls_with_svd(void);
		void calc_delta_thetas_sdls(void);
		void zero_delta_thetas(void);
		void calc_dT_clamped_from_dS(void);
		void scale_back_angle(int j_mode, int method = 0);
};

#endif	/* JACOBIAN_H_ */