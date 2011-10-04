#ifndef NODE_H_
#define NODE_H_

#include "config.h"
#include "linearR3.h"

enum purpose_e {
	JOINT,
	EFFECTOR
};

class Node {
	friend class Tree;

	public:
		Node(const VectorR3 &attach, const VectorR3 &v, int purpose, double minTheta = -M_PI, double maxTheta = M_PI, double restAngle = 0);

		void PrintNode();
		void InitNode();

		//void SetTheta(double newTheta);
		//double GetTheta(void);
		double AddToTheta(double delta);

		const VectorR3& GetAttach(void);
		const VectorR3& GetS(void);
		const VectorR3& GetW(void);

		//double GetMinTheta(void);
		//double GetMaxTheta(void);
		//double GetRestAngle(void);

		void ComputeS(void);
		void ComputeW(void);

		bool IsEffector(void);
		bool IsJoint(void);
		int GetEffectorNum(void);
		int GetJointNum(void);

		bool IsFrozen(void);
		void Freeze(void);
		void UnFreeze(void);

		#ifdef ENABLE_GRAPHICS
		void DrawNode(bool);
		#endif
	private:
		bool freezed;			// Is this node frozen?
		int seqNumJoint;		// sequence number if this node is a joint
		int seqNumEffector;		// sequence number if this node is an effector
		int purpose;			// joint / effector / both
		double theta;			// joint angle (radian)
		//double minTheta;		// lower limit of joint angle
		//double maxTheta;		// upper limit of joint angle
		//double restAngle;		// rest position angle
		VectorR3 attach;		// attachment point
		VectorR3 r;				// relative position vector
		VectorR3 s;				// Global Position
		VectorR3 w;				// Global rotation axis
		VectorR3 v;				// rotation axis
		Node *left;				// left child
		Node *right;			// right sibling
		Node *realparent;		// pointer to real parent
};

#endif	/* NODE_H_ */