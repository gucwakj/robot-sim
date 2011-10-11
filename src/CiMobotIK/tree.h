#ifndef TREE_H_
#define TREE_H_

#include "config.h"
#include "linearR3.h"
#include "node.h"

class Tree {
	public:
		Tree(void);

		void compute(void);
		void init(void);

		void insertRoot(Node *node);
		void insertLeftChild(Node *parent, Node *child);
		void insertRightSibling(Node *parent, Node *child);

		Node* getParent(Node *node);
		Node* getRoot(void);
		Node* getSuccessor(Node *node);
		int getNumNode(void);
		int getNumEffector(void);
		int getNumJoint(void);

		//void unfreeze();
	private:
		Node *root;				// root of tree
		int m_num_effector;		// number of effectors
		int m_num_joint;		// number of joints
		int m_num_node;			// num_node = num_effector + num_joint

		void compute_tree(Node *node);
		void init_tree(Node *node);
		void set_seq_num(Node *node);
		//void unfreeze_tree(Node *node);
};

#endif	/* TREE_H_ */