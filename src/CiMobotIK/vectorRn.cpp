#include "vectorRn.h"

VectorRn::VectorRn(void) {
    this->m_length = 0;
    this->m_alloc_length = 0;
    this->x = 0;
}

VectorRn::VectorRn(int init_length) {
	this->m_length = 0;
	this->m_alloc_length = 0;
	this->x = 0;
	this->setLength(init_length);
}

VectorRn::~VectorRn(void) {
	delete x;
}

void VectorRn::set(const double *d, double scale) {
    double *to = x;
    for ( int i = this->m_length; i > 0; i-- ) {
        *(to++) = (*(d++))*scale;
    }
}

void VectorRn::setLength(int new_length) {
	assert( new_length > 0 );
	if ( new_length > this->m_alloc_length ) {
		delete x;
		this->m_alloc_length = ((new_length > m_alloc_length<<1) ? new_length : m_alloc_length<<1);
		x = new double[m_alloc_length];
	}
	this->m_length = new_length;
}

void VectorRn::setValue(double d) {
    double *target = x;
    for ( int i = this->m_length; i > 0; i-- ) {
        *(target++) = d;
    }
}

void VectorRn::setZero(void) {
	this->setValue(0);
}

// Set the value of the i-th position triple of entries in the vector
void VectorRn::setTriplePosition(int i, const VectorR3& u) {
    int j = 6*i;
    assert( 0 <= j && j + 5 < m_length );
    u.Dump( x + j );
}

// Set the value of the i-th rotation triple of entries in the vector
void VectorRn::setTripleRotation(int i, const VectorR3& v) {
    int j = 6*i;
    assert( 0 <= j && j + 5 < m_length );
    v.Dump( x + j + 3 );
}

void VectorRn::setHextuple(int i, const VectorR3&u, const VectorR3& v) {
    int j = 6*i;
    assert( 0<=j && j+2 < m_length );
    u.Dump( x + j );
    v.Dump( x + j + 3 );
}

int VectorRn::getLength(void) const {
    return this->m_length;
}

double* VectorRn::getPtr(void) {
    return x;
}

const double* VectorRn::getPtr(void) const {
    return x;
}

double* VectorRn::getPtr(int i) {
    assert( 0 <= i && i < this->m_length );
    return (x+i);
}

const double* VectorRn::getPtr(int i) const {
    assert( 0 <= i && i < this->m_length );
    return (x+i);
}

void VectorRn::add(const VectorRn& src, double scale) {
	assert( src.m_length == this->m_length );
	double *to = x;
	double *from = src.x;
	for ( int i = this->m_length; i > 0; i-- ) {
		*(to++) += (*(from++))*scale;
	}
}

double VectorRn::dot(const VectorRn& v) {
    assert( this->getLength() == v.getLength() );
    double res = 0.0;
    const double *p = this->getPtr();
    const double *q = v.getPtr();
    for ( int i = this->m_length; i > 0; i-- ) {
        res += (*(p++))*(*(q++));
    }
    return res;
}

double VectorRn::maxAbs(void) const {
    double result = 0.0;
    double *t = x;
    for ( int i = this->m_length; i > 0; i-- ) {
        if ( (*t) > result ) {
            result = *t;
        }
        else if ( -(*t) > result ) {
            result = -(*t);
        }
        t++;
    }
    return result;
}

double VectorRn::norm(void) {
    return sqrt(this->normSq());
}

double VectorRn::norm(int i) {
    return sqrt(this->normSq(i));
}

double VectorRn::normSq(void) {
    double *target = x;
    double res = 0;
    for ( int i = this->m_length; i > 0; i-- ) {
        res += (*target)*(*target);
        target++;
    }
    return res;
}

double VectorRn::normSq(int i) {
    double *target = x+i;
    double res = 0.0;
    for ( int j = 0; j < 3; j++ ) {
        res += (*target)*(*target);
        target++;
    }
    return res;
}

double& VectorRn::operator[] (int i) {
    assert( 0 <= i && i < this->m_length );
    return *(x+i);
}

const double& VectorRn::operator[] (int i) const {
    assert( 0 <= i && i < this->m_length );
    return *(x+i);
}

VectorRn& VectorRn::operator+= (const VectorRn& src) {
    assert( src.m_length == this->m_length );
    double *to = x;
    double *from = src.x;
    for ( int i = this->m_length; i > 0; i-- ) {
        *(to++) += *(from++);
    }
    return *this;
}

VectorRn& VectorRn::operator-= (const VectorRn& src) {
    assert( src.m_length == this->m_length );
    double *to = x;
    double *from = src.x;
    for ( int i = this->m_length; i > 0; i-- ) {
        *(to++) -= *(from++);
    }
    return *this;
}

VectorRn& VectorRn::operator*= (double f) {
    double *target = x;
    for ( int i = this->m_length; i > 0; i-- ) {
        *(target++) *= f;
    }
    return *this;
}
