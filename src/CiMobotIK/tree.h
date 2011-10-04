#ifndef TREE_H_
#define TREE_H_

#include "config.h"
#include "linearR3.h"
#include "node.h"

class Tree {
	public:
		Tree(void);

		int GetNumNode(void);
		int GetNumEffector(void);
		int GetNumJoint(void);
		void InsertRoot(Node *node);
		void InsertLeftChild(Node *parent, Node *child);
		void InsertRightSibling(Node *parent, Node *child);

		// Accessors based on node numbers
		Node* GetJoint(int);
		Node* GetEffector(int);
		const VectorR3& GetEffectorPosition(int);

		// Accessors for tree traversal
		Node* GetParent( const Node* node ) const;
		Node* GetRoot();
		Node* GetSuccessor ( const Node* ) const;

		void Compute();
		void Print();
		void Init();
		void UnFreeze();

		#ifdef ENABLE_GRAPHICS
		void Draw();
		#endif
	private:
		Node *root;
		int nNode;			// nNode = nEffector + nJoint
		int nEffector;
		int nJoint;

		void SetSeqNum(Node*);
		Node *SearchJoint(Node*, int);
		Node *SearchEffector(Node*, int);
		void ComputeTree(Node*);
		void PrintTree(Node*);
		void InitTree(Node*);
		void UnFreezeTree(Node*);

		#ifdef ENABLE_GRAPHICS
		void DrawTree(Node*);
		#endif
};

#endif	/* TREE_H_ */