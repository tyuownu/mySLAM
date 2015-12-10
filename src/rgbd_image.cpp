#include <rgbd_image.h>
#include <iostream>

namespace mySLAM
{
  // begin RgbdCamera class
  RgbdCamera::RgbdCamera(size_t width, size_t height,
                         const IntrinsicMatrix& intrinsics) :
    width_(width), height_(height), intrinsics_(intrinsics)
  {
    pointcloud_template_.resize(Eigen::NoChange, width_ * height_);
    int idx = 0;

    for(size_t y = 0; y < height_; ++y)
      for(size_t x = 0; x < width_; ++x, ++id)
      {
        pointcloud_template_(0, idx) = (x - intrinsics_.ox()) / intrinsics_.fx();
        pointcloud_template_(1, idx) = (y - intrinsics_.oy()) / intrinsics_.fy();
        pointcloud_template_(2, idx) = 1.0;
        pointcloud_template_(3, idx) = 0.0;
      }
  }
  RgbdCamera::~RgbdCamera() {}

  size_t RgbdCamera::width() const;
  size_t RgbdCamera::height() const;

  RgbdImagePtr RgbdCamera::create(const cv::Mat& intensity,
                                  const cv::Mat& depth) const
  {
    RgbdImagePtr result(new RgbdImage(*this));
    result->intensity = intensity;
    result->depth     = depth;
    result->initialize();

    return result;
  }
  RgbdImagePtr RgbdCamera::create() const
  {
    return boost::make_shared<RgbdImage>(*this);
  }

  void RgbdCamera::buildPointCloud();
  // end RgbdCamera class

  // begin RgbdCameraPyramid class
  RgbdCameraPyramid::RgbdCameraPyramid(const RgbdCamera& base)
  {
    levels_.push_back(boost::make_shared<RgbdCamera>(base));
  }
  RgbdCameraPyramid::RgbdCameraPyramid(size_t width, size_t height,
                                       const mySLAM::IntrinsicMatrix& intrinsics)
  {
    levels_.push_back(boost::make_shared<RgbdCamera>(width, height, intrinsics));
  }
  RgbdCameraPyramid::~RgbdCameraPyramid()
  {}

  RgbdImagePyramidPtr RgbdCameraPyramid::create(const cv::Mat& intensity, const cv::Mat& depth)
  {
    return RgbdImagePyramidPtr(new RgbdImagePyramid(*this, intensity, depth));
  }
  void RgbdCameraPyramid::build(size_t levels)
  {
    size_t start = levels_.size();
    for(size_t i = start; i < levels; i++)
    {
      RgbdCameraPtr& previous = levels_[i-1];
      mySLAM::IntrinsicMatrix intrinsics(previous->intrinsics());
      intrinsics.scale(0.5f);

      levels_.push_back(boost::make_shared<RgbdCamera>(previous->width()/2,
                                                       previous->height()/2,
                                                       intrinsics));
    }
  }
  const RgbdCamera& RgbdCameraPyramid::level(size_t level)
  {
    build(level + 1);
    return *levels_[level];
  }

  const RgbdCamera& RgbdCameraPyramid::level(size_t level) const
  {
    return *levels_[level];
  }
  // end RgbdCameraPyramid class

  // begin RgbdImage class
  RgbdImage::RgbdImage(const RgbdCamera& camera) :
    camera_(camera),
    intensity_requires_calculation_(true),
    depth_requires_calculation_(true),
    pointcloud_requires_build_(true),
    width(0),
    height(0)
  {}
  // end RgbdImage class

  // begin RgbdImagePyramid class
  RgbdImagePyramid::RgbdImagePyramid(RgbdCameraPyramid& camera,
                                     const cv::Mat& intensity,
                                     const cv::Mat& depth)
  {
    levels_.push_back(camera_.level(0).create(intensity, depth));
  }
  RgbdImagePyramid::~RgbdImagePyramid()
  {}

  void RgbdImagePyramid::compute(const size_t num_levels);
  void RgbdImagePyramid::build(const size_t num_levels);

  RgbdImage& level(size_t idx)
  {

  }
  // end RgbdImagePyramid class

  // begin ConvertRawDepthImage class
  ConvertRawDepthImage::ConvertRawDepthImage()
  {
    // nothing
  }

  ConvertRawDepthImage::~ConvertRawDepthImage()
  {
    // nothing
  }

  void ConvertRawDepthImage::convert(const cv::Mat& input, cv::Mat& output, float scale)
  {
    output.create(input.rows, input.cols, CV_32FC1);
    const unsigned short* input_ptr = input.ptr<unsigned short>();
    float* output_ptr = output.ptr<float>();

    for(int idx = 0; idx < input.size().area(); idx++, input_ptr++, output_ptr++)
    {
      if(*input_ptr == 0)
        *output_ptr = std::numeric_limits<float>::quiet_NaN();
      else
        *output_ptr = ((float) *input_ptr) * scale;
    }
  }

  void ConvertRawDepthImage::convertSSE(const cv::Mat& input, cv::Mat& output, float scale)
  {
    // TODO
  }
  // end ConvertRawDepthImage class
}// end namespace mySLAM
