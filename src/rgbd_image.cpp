#include <rgbd_image.h>
#include <iostream>

namespace mySLAM
{
  // some function
  template<typename T>
  static void pyrDownMeanSmooth(const cv::Mat& in, cv::Mat& out)
  {
    out.create(cv::Size(in.size().width/2, in.size().height/2), in.type());
    for(int y = 0; y < out.rows; ++y)
      for(int x = 0; x < out.cols; ++x)
      {
        int x_left   = x * 2;
        int x_right  = x_left + 1;
        int y_top    = y * 2;
        int y_bottom = y_top + 1;

        out.at<T>(y,x) = (T)((in.at<T>(y_top   , x_left ) +
                              in.at<T>(y_top   , x_right) +
                              in.at<T>(y_bottom, x_left ) +
                              in.at<T>(y_bottom, x_right)) / 4.0f);
      }
  }

  template<typename T>
  static void pyrDownSubsample(const cv::Mat& in, cv::Mat& out)
  {
    out.create(cv::Size(in.size().width/2, in.size().height/2), in.type());

    for(int y = 0; y < out.rows; ++y)
      for(int x = 0; x < out.cols; ++x)
      {
        out.at<T>(y,x) = in.at<T>(y * 2, x * 2);
      }
  }
  // end function
  // begin RgbdCamera class
  RgbdCamera::RgbdCamera(size_t width, size_t height,
                         const IntrinsicMatrix& intrinsics) :
    width_(width), height_(height), intrinsics_(intrinsics)
  {
    pointcloud_template_.resize(Eigen::NoChange, width_ * height_);
    int idx = 0;

    for(size_t y = 0; y < height_; ++y)
      for(size_t x = 0; x < width_; ++x, ++idx)
      {
        pointcloud_template_(0, idx) = (x - intrinsics_.ox()) / intrinsics_.fx();
        pointcloud_template_(1, idx) = (y - intrinsics_.oy()) / intrinsics_.fy();
        pointcloud_template_(2, idx) = 1.0;
        pointcloud_template_(3, idx) = 0.0;
      }
  }
  RgbdCamera::~RgbdCamera() {}

  size_t RgbdCamera::width() const
  {
    return width_;
  }
  size_t RgbdCamera::height() const
  {
    return height_;
  }

  const mySLAM::IntrinsicMatrix& RgbdCamera::intrinsics() const
  {
    return intrinsics_;
  }

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

  bool RgbdCamera::hasSameSize(const cv::Mat& img) const
  {
    return img.cols == width_ && img.rows == height_;
  }

  void RgbdCamera::buildPointCloud(const cv::Mat& depth, PointCloud& pointcloud) const
  {
    assert(hasSameSize(depth));
    pointcloud.resize(Eigen::NoChange, width_ * height_);
    const float* depth_ptr = depth.ptr<float>();
    int id = 0;
    for(size_t y = 0; y < height_; y++)
      for(size_t x = 0; x < width_; x++, depth_ptr++, id++)
      {
        pointcloud.col(id) = pointcloud_template_.col(id) * (*depth_ptr);
        pointcloud(3, id) = 1.0f;
      }
  }
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
  RgbdImage::~RgbdImage()
  {}

  const RgbdCamera& RgbdImage::camera() const {}

  void RgbdImage::initialize()
  {
    assert(hasIntensity() || hasDepth());
    if(hasIntensity() && hasDepth())
    {
      assert(intensity.size() == depth.size());
    }

    if(hasIntensity())
    {
      assert(intensity.type() ==
             cv::DataType<IntensityType>::type &&
             depth.channels() == 1);
      width  = intensity.cols;
      height = intensity.rows;
    }

    if(hasDepth())
    {
      assert(depth.type() == cv::DataType<DepthType>::type && depth.channels() == 1);
      width  = depth.cols;
      height = depth.rows;
    }

    intensity_requires_calculation_ = true;
    depth_requires_calculation_     = true;
    pointcloud_requires_build_      = true;
  }

  bool RgbdImage::hasIntensity() const
  {
    return !intensity.empty();
  }

  bool RgbdImage::hasRgb() const
  {
    return !rgb.empty();
  }

  bool RgbdImage::hasDepth() const
  {
    return !depth.empty();
  }

  void RgbdImage::calculateDerivatives()
  {
    calculateIntensityDerivatives();
    calculateDepthDerivatives();
  }

  bool RgbdImage::calculateIntensityDerivatives()
  {
    if(!intensity_requires_calculation_) return false;
    assert(hasIntensity());

    calculateDerivativeX<IntensityType>(intensity, intensity_dx);
    calculateDerivativeY<IntensityType>(intensity, intensity_dy);

    intensity_requires_calculation_ = false;
    return true;
  }

  void RgbdImage::calculateDepthDerivatives()
  {
    if(!depth_requires_calculation_) return ;
    assert(hasDepth());
    calculateDerivativeX<DepthType>(depth, depth_dx);
    calculateDerivativeY<DepthType>(depth, depth_dy);
    depth_requires_calculation_ = false;
  } // TODO: can use SSE ??

  template<typename T>
  void RgbdImage::calculateDerivativeX(const cv::Mat& img, cv::Mat& result)
  {
    result.create(img.size(), img.type());
    for(int y = 0; y < img.rows; y++)
      for(int x = 0; x < img.cols; x++)
      {
        int prev = std::max(x - 1, 0);
        int next = std::min(x + 1, img.cols - 1);
        result.at<T>(y, x) = (T) (img.at<T>(y, next) - img.at<T>(y, prev)) * 0.5f;
      }
  }

  template<typename T>
  void RgbdImage::calculateDerivativeY(const cv::Mat& img, cv::Mat& result)
  {
    result.create(img.size(), img.type());
    for(int y = 0; y < img.rows; y++)
      for(int x = 0; x < img.cols; x++)
      {
        int prev = std::max(y - 1, 0);
        int next = std::min(y + 1, img.rows - 1);
        result.at<T>(y, x) = (T) (img.at<T>(next, x) - img.at<T>(prev, x)) * 0.5f;
      }
  }

  void RgbdImage::buildPointCloud()
  {
    if(!pointcloud_requires_build_) return ;
    assert(!hasDepth());
    camera_.buildPointCloud(depth, pointcloud);
    pointcloud_requires_build_ = false;
  }

  void RgbdImage::buildAccelerationStructure()
  {
    if(0 == acceleration.total())
    {
      calculateDerivatives();
      cv::Mat zeros = cv::Mat::zeros(intensity.size(), intensity.type());
      cv::Mat channels[8] = {intensity, depth, intensity_dx, intensity_dy, depth_dx, depth_dy, zeros, zeros};
      cv::merge(channels, 8, acceleration);
    }
  }

  bool RgbdImage::inImage(const float& x, const float& y) const
  {
    return x >= 0&& x < width && y >= 0 && y < height;
  }

  // end RgbdImage class

  // begin RgbdImagePyramid class
  RgbdImagePyramid::RgbdImagePyramid(RgbdCameraPyramid& camera,
                                     const cv::Mat& intensity,
                                     const cv::Mat& depth) :
    camera_(camera)
  {
    levels_.push_back(camera_.level(0).create(intensity, depth));
  }
  RgbdImagePyramid::~RgbdImagePyramid()
  {}

  void RgbdImagePyramid::compute(const size_t num_levels)
  {
    build(num_levels); // TODO: maybe can delete it.
  }
  void RgbdImagePyramid::build(const size_t num_levels)
  {
    if (levels_.size() >= num_levels) return ;

    for (size_t it = levels_.size(); it < num_levels; ++it)
    {
      levels_.push_back(camera_.level(it).create());
      pyrDownMeanSmooth<IntensityType>(levels_[it - 1]->intensity,
                                       levels_[it - 1]->intensity);
      pyrDownSubsample<float>(levels_[it - 1]->depth, levels_[it]->depth);
      levels_[it]->initialize();
    }
  }

  RgbdImage& RgbdImagePyramid::level(size_t idx)
  {
    assert(idx < levels_.size());
    return *levels_[idx];
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
