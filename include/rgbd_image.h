#ifndef _RGBD_IMAGE_H_
#define _RGBD_IMAGE_H_

#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <boost/make_shared.hpp>
#include <boost/smart_ptr.hpp>

#include <intrinsic_matrix.h>
#include <data_types.h>

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
    virtual ~RgbdCamera(); //need this virtual?

    size_t width() const;
    size_t height() const;

    const mySLAM::IntrinsicMatrix& intrinsics() const;

    RgbdImagePtr create(const cv::Mat& intensity, const cv::Mat& depth) const;
    RgbdImagePtr create() const;

    void buildPointCloud(const cv::Mat& depth, PointCloud& pointcloud) const;

  private:
    size_t width_, height_;
    bool hasSameSize(const cv::Mat& img) const;
    mySLAM::IntrinsicMatrix intrinsics_;
    PointCloud pointcloud_template_;
  };

  typedef boost::shared_ptr<RgbdCamera> RgbdCameraPtr;
  typedef boost::shared_ptr<const RgbdCamera> RgbdCameraConstPtr;

  class RgbdCameraPyramid
  {
  public:
    RgbdCameraPyramid(const RgbdCamera& base);
    RgbdCameraPyramid(size_t base_width, size_t base_height, const mySLAM::IntrinsicMatrix& base_intrinsics);

    ~RgbdCameraPyramid();  // need virtual?

    RgbdImagePyramidPtr create(const cv::Mat& base_intensity, const cv::Mat& base_depth);

    void build(size_t levels);
    const RgbdCamera& level(size_t level);
    const RgbdCamera& level(size_t level) const;

  private:
    std::vector<RgbdCameraPtr> levels_;
  };

  typedef boost::shared_ptr<RgbdCameraPyramid> RgbdCameraPyramidPtr;
  typedef boost::shared_ptr<const RgbdCameraPyramid> RgbdCameraPyramidConstPtr;

  class RgbdImage
  {
  public:
    RgbdImage(const RgbdCamera& camera);
    virtual ~RgbdImage();

    typedef mySLAM::PointCloud PointCloud;

    const RgbdCamera& camera() const;

    cv::Mat intensity, intensity_dx, intensity_dy;

    cv::Mat depth, depth_dx, depth_dy;

    cv::Mat normals, angles;  // TODO, what that for?

    cv::Mat rgb;
    PointCloud pointcloud;

    typedef cv::Vec<float, 8> Vec8f;
    cv::Mat_<Vec8f> acceleration;

    size_t width, height;
    double timestamp;

    bool hasIntensity() const; // TODO, for what?
    bool hasDepth() const;
    bool hasRgb() const;

    void initialize();

    // calculate derivatives. but why there is a bool-return?
    void calculateDerivatives();
    bool calculateIntensityDerivatives();
    void calculateDepthDerivatives();

    void calculateNormals();
    void buildPointCloud();

    void buildAccelerationStructure();

    void warpIntensity(const AffineTransform& transformation,
                       const PointCloud& reference_pointcloud,
                       const IntrinsicMatrix& intrinsics,
                       RgbdImage& result, PointCloud& transformed_pointcloud);
    // SSE version to be continue.
    void warpIntensitySSE(const AffineTransform& transformation,
                       const PointCloud& reference_pointcloud,
                       const IntrinsicMatrix& intrinsics,
                       RgbdImage& result, PointCloud& transformed_pointcloud);
    // SSE version without warped pointcloud, what is that?
    void warpIntensitySSE(const AffineTransform& transformation,
                       const PointCloud& reference_pointcloud,
                       const IntrinsicMatrix& intrinsics,
                       RgbdImage& result);

    void warpIntensityForward(const AffineTransform& transformation,
                              const IntrinsicMatrix& intrinsics,
                              RgbdImage& result, cv::Mat_<cv::Vec3d>& cloud);
    void warpDepthForward(const AffineTransform& transformation,
                          const IntrinsicMatrix& intrinsics,
                          RgbdImage& result, cv::Mat_<cv::Vec3d>& cloud);
    void warpDepthForwardAdvanced(const AffineTransform& transformation,
                                  const IntrinsicMatrix& intrinsics,
                                  RgbdImage& result);

    bool inImage(const float&x, const float& y) const;

  private:
    bool intensity_requires_calculation_,
      depth_requires_calculation_,
      pointcloud_requires_build_;

    const RgbdCameraPyramid& camera_;

    template<typename T>
      void calculateDerivativeX(const cv::Mat& img,
                                cv::Mat& result); //adding SSE version?
    template<typename T>
      void calculateDerivativeY(const cv::Mat& img,
                                cv::Mat& result);

    enum WarpIntensityOptions
    {
      WithPointCloud,
      WithoutPointCloud
    };

    template<int PointCloudOption>
      void warpIntensitySseImpl(const AffineTransform& transformation,
                                const PointCloud& reference_pointcloud,
                                const IntrinsicMatrix& intrinsics,
                                RgbdImage& result,
                                PointCloud& transformed_pointcloud);
  };

  class RgbdImagePyramid
  {
  public:
    typedef boost::shared_ptr<mySLAM::RgbdImagePyramid> Ptr;

    RgbdImagePyramid(RgbdCameraPyramid& camera,
                     const cv::Mat& intensity,
                     const cv::Mat& depth);
    virtual ~RgbdImagePyramid();

    void compute(const size_t num_levels);
    void build(const size_t num_levels);

    RgbdImage& level(size_t idx);
    double timestamp() const;

  private:
    RgbdCameraPyramid& camera_;
    std::vector<RgbdImagePtr> levels_;
  };

  class ConvertRawDepthImage
  {
    // Converts the raw depth image(CV_16U->CV_32F) image and replacing 0 with NaNs;
  public:
    ConvertRawDepthImage();
    virtual ~ConvertRawDepthImage();

    static void convert   (const cv::Mat& input, cv::Mat& output, float scale);
    static void convertSSE(const cv::Mat& input, cv::Mat& output, float scale);
    // TODO
  };

} // end namespace mySLAM
#endif
