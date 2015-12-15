#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "keyframe_tracker.h"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

namespace mySLAM
{
  class KeyframeTracker::Impl
  {
  public:
    friend class ::mySLAM::KeyframeTracker;
    Impl();
    ~Impl();

    void init(const Eigen::Affine3d& initial_transformation);
  private:
    Eigen::Affine3d initial_transformation_, relative_transformation_, last_absolute_transformation_;
    mySLAM::RgbdImagePyramid::Ptr previous_;
    mySLAM::KeyframeTrackerConfig cfg_;
  };
  // begin Impl function
  void KeyframeTracker::Impl::init(const Eigen::Affine3d& initial_transformation)
  {
    initial_transformation_ = initial_transformation;
    relative_transformation_.setIdentity();
  }
  // end Impl function

  void KeyframeTracker::init( )
  {
    init(Eigen::Affine3d::Identity());
  }

  void KeyframeTracker::init(const Eigen::Affine3d& initial_transformation)
  {
    impl_->init(initial_transformation);
  }
  void KeyframeTracker::configureTracking(const mySLAM::DenseTracker::config& cfg)
  {
    impl_
  }
}  // end namespace mySLAM
