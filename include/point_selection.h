#ifndef _POINT_SELECTION_H
#define _POINT_SELECTION_H

#include <intrinsic_matrix.h>
#include <rgbd_image.h>

namespace mySLAM
{
  class PointSelectionPredicate
  {
  public:
    virtual ~PointSelectionPredicate() {}

    virtual bool isPointOK(const size_t& x,
                           const size_t& y,
                           const size_t& z,
                           const float& idx,
                           const float& idy,
                           const float& zdx,
                           const float& zdy) const = 0;
  };

  // class ValidPointPredicate is useless?
  class ValidPointPredicate : public PointSelectionPredicate
  {
  public:
    virtual ~ValidPointPredicate() {}
    virtual bool isPointOK(const size_t& x,
                           const size_t& y,
                           const size_t& z,
                           const float& idx,
                           const float& idy,
                           const float& zdx,
                           const float& zdy) const
    {
      return z == z && zdx == zdx && zdy == zdy;
    }
  };

  // mostly we use this
  class ValidPointAndGradientThresholdPredicate : public PointSelectionPredicate
  {
  public:
    float intensity_threshold, depth_threshold;
    ValidPointAndGradientThresholdPredicate() :
      intensity_threshold(0.0f),
      depth_threshold(0.0f)
    {}
    virtual ~ValidPointAndGradientThresholdPredicate() {}

    virtual bool isPointOK(const size_t& x,
                           const size_t& y,
                           const size_t& z,
                           const float& idx,
                           const float& idy,
                           const float& zdx,
                           const float& zdy) const
    {
      return z == z && zdx == zdx && zdy == zdy &&
        (std::abs(idx) > intensity_threshold ||
         std::abs(idy) > intensity_threshold ||
         std::abs(zdx) > depth_threshold ||
         std::abs(zdy) > depth_threshold);
    }
  };


  class PointSelection
  {
  public:
    typedef PointWithIntensityAndDepth::VectorType PointVector;
    typedef PointVector::iterator PointIterator;

    PointSelection(const PointSelectionPredicate& predicate);
    PointSelection(mySLAM::RgbdImagePyramid& pyramid, const PointSelectionPredicate& predicate);
    virtual ~PointSelection();

    mySLAM::RgbdImagePyramid& getRgbdImagePyramid();
    void setRgbdImagePyramid(mySLAM::RgbdImagePyramid& pyramid);
    size_t getMaximumNumberOfPoint(const size_t& level);
    void select(const size_t& level, PointIterator& first_point, PointIterator& last_point);
    void recycle(mySLAM::RgbdImagePyramid& pyramid);
    bool getDebugIndex(const size_t& level, cv::Mat& dbg_idx);
    void debug(bool v)
    {
    debug_ = v;
    }
    bool debug() const
    {
    return debug_;
    }

  private:
    struct Storage
    {
    public:
      PointVector points;
      PointIterator points_end;
      bool is_cached;

      cv::Mat debug_idx;
      Storage();
      void allocate(size_t max_points);
    };

    mySLAM::RgbdImagePyramid *pyramid_;
    std::vector<Storage> storage_;
    const PointSelectionPredicate& predicate_;
    bool debug_;

    PointIterator selectPointsFromImage(const mySLAM::RgbdImage& img,
                                        const PointIterator& first_point,
                                        const PointIterator& last_point,
                                        cv::Mat& debug_idx);
  };

}  // end namespace mySLAM


#endif
