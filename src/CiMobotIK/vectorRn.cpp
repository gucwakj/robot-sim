#include "vectorRn.h"

VectorRn VectorRn::WorkVector;

double VectorRn::MaxAbs(void) const {
	double result = 0.0;
	double* t = x;
	for ( long i = length; i>0; i-- ) {
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

VectorRn::VectorRn() {
	length = 0;
	AllocLength = 0;
	x = 0;
}

VectorRn::VectorRn( long initLength ) {
	length = 0;
	AllocLength = 0;
	x = 0;
	SetLength( initLength );
}

VectorRn::~VectorRn() {
	delete x;
}

// Resize.
// If the array is shortened, the information about the allocated length is lost.
void VectorRn::SetLength( long newLength ) {
	assert ( newLength > 0 );
	if ( newLength > AllocLength ) {
		delete x;
		AllocLength = ((newLength > AllocLength<<1) ? newLength : AllocLength<<1);
		x = new double[AllocLength];
	}
	length = newLength;
}

// Zero out the entire vector
void VectorRn::SetZero() {
	double* target = x;
	for ( long i=length; i>0; i-- ) {
		*(target++) = 0.0;
	}
}

// Set the value of the i-th triple of entries in the vector
void VectorRn::SetTriple( long i, const VectorR3& u ) {
	long j = 3*i;
	assert ( 0<=j && j+2 < length );
	u.Dump( x+j );
}

void VectorRn::Fill( double d ) {
	double *to = x;
	for ( long i=length; i>0; i-- ) {
		*(to++) = d;
	}
}

void VectorRn::Load( const double* d ) {
	double *to = x;
	for ( long i=length; i>0; i-- ) {
		*(to++) = *(d++);
	}
}

void VectorRn::LoadScaled( const double* d, double scaleFactor ) {
	double *to = x;
	for ( long i=length; i>0; i-- ) {
		*(to++) = (*(d++))*scaleFactor;
	}
}

void VectorRn::Set( const VectorRn& src ) {
	assert ( src.length == this->length );
	double* to = x;
	double* from = src.x;
	for ( long i=length; i>0; i-- ) {
		*(to++) = *(from++);
	}
}

VectorRn& VectorRn::operator+=( const VectorRn& src ) {
	assert ( src.length == this->length );
	double* to = x;
	double* from = src.x;
	for ( long i=length; i>0; i-- ) {
		*(to++) += *(from++);
	}
	return *this;
}

VectorRn& VectorRn::operator-=( const VectorRn& src ) {
	assert ( src.length == this->length );
	double* to = x;
	double* from = src.x;
	for ( long i=length; i>0; i-- ) {
		*(to++) -= *(from++);
	}
	return *this;
}

void VectorRn::AddScaled (const VectorRn& src, double scaleFactor ) {
	assert ( src.length == this->length );
	double* to = x;
	double* from = src.x;
	for ( long i=length; i>0; i-- ) {
		*(to++) += (*(from++))*scaleFactor;
	}
}

VectorRn& VectorRn::operator*=( double f ) {
	double* target = x;
	for ( long i=length; i>0; i-- ) {
		*(target++) *= f;
	}
	return *this;
}

double VectorRn::NormSq() const {
	double* target = x;
	double res = 0.0;
	for ( long i=length; i>0; i-- ) {
		res += (*target)*(*target);
		target++;
	}
	return res;
}

double VectorRn::dot(const VectorRn& v) {
    assert( this->GetLength() == v.GetLength() );
    double res = 0.0;
    const double *p = this->GetPtr();
    const double *q = v.GetPtr();
    for ( int i = this->length; i > 0; i-- ) {
        res += (*(p++))*(*(q++));
    }
    return res;
}
