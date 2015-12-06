#include <intrinsic_matrix.h>

namespace mySLAM
{
  IntrinsicMatrix::IntrinsicMatrix(const IntrinsicMatrix& other)
  {

  }
  IntrinsicMatrix IntrinsicMatrix::create(float fx, float fy, float ox, float oy)
  {
    IntrinsicMatrix result;

    result.data.setZero();
    result.data(0,0) = fx;
    result.data(1,1) = fy;
    result.data(0,2) = ox;
    result.data(1,2) = oy;
    result.data(2,2) = 1.0f;

    return result;
  }

  float IntrinsicMatrix::fx() const
  { return data(0,0); }
  float IntrinsicMatrix::fy() const
  { return data(1,1); }
  float IntrinsicMatrix::ox() const
  { return data(0,2); }
  float IntrinsicMatrix::oy() const
  { return data(1,2); }


}
