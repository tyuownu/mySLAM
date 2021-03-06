#ifndef _DATA_TYPES_H
#define _DATA_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace mySLAM
{
  typedef float IntensityType;
  static const IntensityType Invalid = std::numeric_limits<IntensityType>::quiet_NaN();

  typedef float DepthType;
  static const DepthType InvalidDepth = std::numeric_limits<DepthType>::quiet_NaN();

  typedef float NumType;

  typedef Eigen::Matrix<NumType, 6, 6> Matrix6x6;
  typedef Eigen::Matrix<NumType, 1, 2> Matrix1x2;
  typedef Eigen::Matrix<NumType, 2, 6> Matrix2x6;

  typedef Eigen::Matrix<NumType, 6, 1> Vector6;
  typedef Eigen::Matrix<NumType, 4, 1> Vector4;

  typedef Eigen::Transform<NumType, 3, Eigen::Affine> AffineTransform;

  typedef Eigen::Affine3d AffineTransformd;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
} // end namespace mySLAM

#endif
