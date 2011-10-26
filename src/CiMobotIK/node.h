#ifndef NODE_H_
#define NODE_H_

//#include "config.h"
#include "matrixR33.h"
#include "vectorR3.h"

enum purpose_e {
	JOINT,
	EFFECTOR
};

class Node {
	friend class Tree;
	public:
        Node(const VectorR3 &s_init, const VectorR3 &w_init, int purpose, double minTheta = -M_PI, double maxTheta = M_PI, double initAngle = 0);
        Node(const VectorR3 &s_init, const VectorR3 &w_init, const MatrixR33 &R_init, int purpose, double theta_init = 0, double theta_min = -M_PI, double theta_max = M_PI);

		void initNode(void);

		double updateTheta(double delta);

		void setSeqNum(int seq_num);
		void setFrozen(bool s);

        const MatrixR33& getR(void);
        const MatrixR33& getRInit(void);
		const VectorR3& getS(void);
		const VectorR3& getSInit(void);
		const VectorR3& getW(void);
		const VectorR3& getWInit(void);
		double getTheta(void);
		double getThetaInit(void);
		double getThetaMin(void);
		double getThetaMax(void);
		int getEffectorNum(void);
		int getJointNum(void);
		int getPurpose(void);

		void computeS(void);
        void computeW(void);
        void computeR(void);

		bool isEffector(void);
		bool isJoint(void);
		bool isFrozen(void);
	private:
		bool m_frozen;			// Is this node frozen?
		int m_seq_num_joint;	// sequence number if this node is a joint
		int m_seq_num_effector;	// sequence number if this node is an effector
		int m_purpose;			// joint / effector / both
		double m_theta;			// joint angle (radian)
		double m_theta_init;	// initial theta angle of joint
		double m_theta_min;		// lower limit of joint angle
		double m_theta_max;		// upper limit of joint angle
		MatrixR33 m_R;          // global rotation matrix
		MatrixR33 m_R_init;     // global rotation matrix when joint is zero
		VectorR3 m_r;			// relative position vector
		VectorR3 m_s;			// global position
		VectorR3 m_s_init;		// global position when joint is zero
		VectorR3 m_w;			// global rotation axis
		VectorR3 m_w_init;		// global rotation axis when joint is zero
		Node *left;				// left child
		Node *right;			// right sibling
		Node *realparent;		// real parent
};

#endif	/* NODE_H_ */