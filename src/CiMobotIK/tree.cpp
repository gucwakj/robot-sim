#include <iostream>
#include "linearR3.h"
#include "tree.h"
#ifdef ENABLE_GRAPHICS
	#include <GL/gl.h>
#endif

Tree::Tree(void) {
	this->root = 0;
	this->nNode = 0;
	this->nEffector = 0;
	this->nJoint = 0;
}

// Initialize all nodes in the tree
void Tree::Init(void) {
	InitTree(root);
}

void Tree::SetSeqNum(Node *node) {
	switch (node->purpose) {
		case JOINT:
			node->seqNumJoint = this->nJoint++;
			node->seqNumEffector = -1;
			break;
		case EFFECTOR:
			node->seqNumJoint = -1;
			node->seqNumEffector = this->nEffector++;
			break;
	}
}

void Tree::InsertRoot(Node *root) {
	assert(nNode==0);
	this->nNode++;
	this->root = root;
	root->r = root->attach;
	assert( !(root->left || root->right) );
	SetSeqNum(root);
}

void Tree::InsertLeftChild(Node *parent, Node *child) {
	assert(parent);
	nNode++;
	parent->left = child;
	child->realparent = parent;
	child->r = child->attach - child->realparent->attach;
	assert( !(child->left || child->right) );
	SetSeqNum(child);
}

void Tree::InsertRightSibling(Node *parent, Node *child) {
	assert(parent);
	nNode++;
	parent->right = child;
	child->realparent = parent->realparent;
	child->r = child->attach - child->realparent->attach;
	assert( !(child->left || child->right) );
	SetSeqNum(child);
}

int Tree::GetNumNode(void) {
	return this->nNode;
}

int Tree::GetNumEffector(void) {
	return this->nEffector;
}

int Tree::GetNumJoint(void) {
	return this->nJoint;
}

// Search recursively below "node" for the node with index value.
Node* Tree::SearchJoint(Node *node, int index) {
	if (node != 0) {
		if (node->seqNumJoint == index)
			return node;
		else
			return SearchJoint(node->right, index);
	}
	else {
		return NULL;
	}
}

// Search recursively below node for the end effector with the index value
Node* Tree::SearchEffector(Node *node, int index) {
	if (node != 0) {
		if (node->seqNumEffector == index)
			return node;
		else
			return SearchJoint(node->right, index);
	} else {
		return NULL;
	}
}

// Get the joint with the index value
Node* Tree::GetJoint(int index) {
	return SearchJoint(root, index);
}

// Get the end effector for the index value
Node* Tree::GetEffector(int index) {
	return SearchEffector(root, index);
}

// Returns the global position of the effector.
const VectorR3& Tree::GetEffectorPosition(int index) {
	Node* effector = GetEffector(index);
	assert(effector);
	return (effector->s);
}

Node* Tree::GetParent( const Node* node ) const {
	return node->realparent;
}

Node* Tree::GetRoot() {
	return root;
}

Node* Tree::GetSuccessor( const Node* node ) const {
	if ( node->left ) {
		return node->left;
	}
	while ( true ) {
		if ( node->right ) {
			return ( node->right );
		}
		node = node->realparent;
		if ( !node ) {
			return 0;		// Back to root, finished traversal
		}
	}
}

void Tree::ComputeTree(Node *node) {
	if (node != 0) {
		node->ComputeS();
		node->ComputeW();
		ComputeTree(node->left);
		ComputeTree(node->right);
	}
}

void Tree::Compute(void) {
	ComputeTree(root);
}

void Tree::PrintTree(Node* node) {
	if (node != 0) {
		node->PrintNode();
		PrintTree(node->left);
		PrintTree(node->right);
	}
}

void Tree::Print(void) {
	PrintTree(root);
	cout << "\n";
}

// Recursively initialize tree below the node
void Tree::InitTree(Node* node) {
	if (node != 0) {
		node->InitNode();
		InitTree(node->left);
		InitTree(node->right);
	}
}


void Tree::UnFreezeTree(Node* node) {
	if (node != 0) {
		node->UnFreeze();
		UnFreezeTree(node->left);
		UnFreezeTree(node->right);
	}
}

void Tree::UnFreeze(void) {
	UnFreezeTree(root);
}

#ifdef ENABLE_GRAPHICS
void Tree::DrawTree(Node* node) {
	if (node != 0) {
		glPushMatrix();
		node->DrawNode( node==root );	// Recursively draw node and update ModelView matrix
		if (node->left) {
			DrawTree(node->left);		// Draw tree of children recursively
		}
		glPopMatrix();
		if (node->right) {
			DrawTree(node->right);		// Draw right siblings recursively
		}
	}
}

void Tree::Draw(void) {
	DrawTree(root);
}
#endif
