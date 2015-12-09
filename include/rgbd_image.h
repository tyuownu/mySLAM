#ifndef _RGBD_IMAGE_H_
#define _RGBD_IMAGE_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/smart_ptr.hpp>

#include <intrinsic_matrix.h>

namespace mySLAM
{
  typedef Eigen::Matrix<float, 8, 1> Vector8f;
  struct EIGEN_ALIGN16 PointWithIntensityAndDepth
  {
    typedef EIGEN_ALIGN16 union
    {
      float data[4];
      struct
      {
        float x, y, z;
      };
    } Point;

    typedef EIGEN_ALIGN16 union
    {
      float data[8];
      struct
      {
        float i, z, idx, idy, zdx, zdy, time_interpolation;
      };
      //what does the time_interpolation means?
    } IntensityAndDepth;

    typedef std::vector<PointWithIntensityAndDepth, Eigen::aligned_allocator<PointWithIntensityAndDepth> > VectorType;

    Point point;
    IntensityAndDepth intensity_and_depth;

    Eigen::Vector4f::AlignedMapType getPointVec4f()
    {
      return Eigen::Vector4f::AlignedMapType(point.data);
    }

    Eigen::Vector2f::AlignedMapType getIntensityAndDepthVec2f()
    {
      return Eigen::Vector2f::AlignedMapType(intensity_and_depth.data);
    }

    Eigen::Vector2f::MapType getIntensityDerivativeVec2f()
    {
      return Eigen::Vector2f::MapType(intensity_and_depth.data + 2);
    }

    Eigen::Vector2f::MapType getDepthDerivativeVec2f()
    {
      return Eigen::Vector2f::MapType(intensity_and_depth.data + 4);
    }

    Vector8f::AlignedMapType getIntensityAndDepthWithDerivativesVec8f()
    {
      return Vector8f::AlignedMapType(intensity_and_depth.data);
    }
  };

  typedef Eigen::Matrix<float, 4, Eigen::Dynamic, Eigen::ColMajor> PointCloud;
  class RgbdImage;
  typedef boost::shared_ptr<RgbdImage> RgbdImagePtr;

  class RgbdImagePyramid;
  typedef boost::shared_ptr<RgbdImagePyramid> RgbdImagePyramidPtr;

  class RgbdCamera
  {
  public:
    RgbdCamera(size_t width, size_t height, const mySLAM::IntrinsicMatrix& intrinsics);
    virtual ~RgbdCamera();
  }

}
