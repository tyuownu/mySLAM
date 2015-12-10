#ifndef _INTRINSIC_MATRIX_H_
#define _INTRINSIC_MATRIX_H_

#include <Eigen/Core>

namespace mySLAM
{
  class IntrinsicMatrix
  {
  public:
    IntrinsicMatrix() {}
    IntrinsicMatrix(const IntrinsicMatrix& other);

    IntrinsicMatrix create(float fx, float fy, float ox, float oy);
    float fx() const;
    float fy() const;
    float ox() const;
    float oy() const;

    void scale(float factor);

    Eigen::Matrix3f data;

  }
}

#endif
