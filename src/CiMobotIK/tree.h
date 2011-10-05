#ifndef TREE_H_
#define TREE_H_

#include "config.h"
#include "linearR3.h"
#include "node.h"

class Tree {
	public:
		Tree(void);

		void init(void);
		void compute(void);

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

		#ifdef ENABLE_GRAPHICS
		void Draw();
		#endif
	private:
		Node *root;
		int m_num_node;			// nNode = nEffector + nJoint
		int nEffector;
		int m_num_effector;
		int nJoint;
		int m_num_joint;

		void compute_tree(Node *node);
		void init_tree(Node *node);
		void set_seq_num(Node *node);
		//void unfreeze_tree(Node *node);

		#ifdef ENABLE_GRAPHICS
		void DrawTree(Node*);
		#endif
};

#endif	/* TREE_H_ */