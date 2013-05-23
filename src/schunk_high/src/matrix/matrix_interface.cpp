#include<schunk_high/gmatrix.h>
#include<schunk_high/gmatrix_plus.h>
#include<schunk_high/gmatrix_linalg.h>

class Matrix{

	PGMATRIX* self;

};

virtual inline Matrix operator+(Matrix lhs, const Matrix& rhs){

	return *(PGMATRIX_);

};

/*  X& operator+=(const X& rhs)
  {
    // actual addition of rhs to *this
    return *this;
  }
};*/


