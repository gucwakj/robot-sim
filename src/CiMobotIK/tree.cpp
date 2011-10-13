#include <iostream>
#include "tree.h"

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
	root->m_r = root->m_s_init;
	assert( !(root->left || root->right) );
	this->set_seq_num(root);
}

void Tree::insertLeftChild(Node *parent, Node *child) {
	assert(parent);
	this->m_num_node++;
	parent->left = child;
	child->realparent = parent;
	child->m_r = child->m_s_init - child->realparent->m_s_init;
	assert( !(child->left || child->right) );
	this->set_seq_num(child);
}

void Tree::insertRightSibling(Node *parent, Node *child) {
	assert(parent);
	this->m_num_node++;
	parent->right = child;
	child->realparent = parent->realparent;
	child->m_r = child->m_s_init - child->realparent->m_s_init;
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
			return 0;
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
		node->computeS();
		node->computeW();
		this->compute_tree(node->left);
		this->compute_tree(node->right);
	}
}

void Tree::init_tree(Node *node) {
	if (node != 0) {
		node->initNode();
		this->init_tree(node->left);
		this->init_tree(node->right);
	}
}

void Tree::set_seq_num(Node *node) {
	switch ( node->getPurpose() ) {
		case JOINT:
			node->setSeqNum(this->m_num_joint);
			this->m_num_joint++;
			break;
		case EFFECTOR:
			node->setSeqNum(this->m_num_effector);
			this->m_num_effector++;
			break;
	}
}
