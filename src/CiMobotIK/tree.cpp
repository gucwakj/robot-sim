#include <iostream>
#include "tree.h"
#ifdef ENABLE_GRAPHICS
#include <GL/gl.h>
#endif

Tree::Tree(void) {
	this->root = 0;
	this->m_num_node = 0;
	this->m_num_effector = 0;
	this->m_num_joint = 0;
}

void Tree::init(void) {
	this->init_tree(this->root);
}

void Tree::compute(void) {
	this->compute_tree(this->root);
}

void Tree::insertRoot(Node *root) {
	assert(m_num_node==0);
	this->m_num_node++;
	this->root = root;
	root->r = root->attach;
	assert( !(root->left || root->right) );
	this->set_seq_num(root);
}

void Tree::insertLeftChild(Node *parent, Node *child) {
	assert(parent);
	m_num_node++;
	parent->left = child;
	child->realparent = parent;
	child->r = child->attach - child->realparent->attach;
	assert( !(child->left || child->right) );
	this->set_seq_num(child);
}

void Tree::insertRightSibling(Node *parent, Node *child) {
	assert(parent);
	m_num_node++;
	parent->right = child;
	child->realparent = parent->realparent;
	child->r = child->attach - child->realparent->attach;
	assert( !(child->left || child->right) );
	this->set_seq_num(child);
}

Node* Tree::getParent(Node *node) {
	return node->realparent;
}

Node* Tree::getRoot(void) {
	return this->root;
}

Node* Tree::getSuccessor(Node *node) {
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

int Tree::getNumNode(void) {
	return this->m_num_node;
}

int Tree::getNumEffector(void) {
	return this->m_num_effector;
}

int Tree::getNumJoint(void) {
	return this->m_num_joint;
}

void Tree::compute_tree(Node *node) {
	if (node != 0) {
		node->ComputeS();
		node->ComputeW();
		this->compute_tree(node->left);
		this->compute_tree(node->right);
	}
}

void Tree::init_tree(Node *node) {
	if (node != 0) {
		node->InitNode();
		this->init_tree(node->left);
		this->init_tree(node->right);
	}
}

void Tree::set_seq_num(Node *node) {
	switch (node->purpose) {
		case JOINT:
			node->seqNumJoint = this->m_num_joint++;
			node->seqNumEffector = -1;
			break;
		case EFFECTOR:
			node->seqNumJoint = -1;
			node->seqNumEffector = this->m_num_effector++;
			break;
	}
}

/*void Tree::UnFreeze(void) {
	this->unfreeze_tree(root);
}*/

/*void Tree::unfreeze_tree(Node *node) {
	if (node != 0) {
		node->UnFreeze();
		this->unfreeze_tree(node->left);
		this->unfreeze_tree(node->right);
	}
}*/

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
