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
    void update(const mySLAM::RgbdImagePyramid::Ptr& current, Eigen::Affine3d& absolute_transformation);
  private:
    KeyframeGraph graph_;
    LocalTracker lt_;
    Eigen::Affine3d initial_transformation_, relative_transformation_, last_absolute_transformation_;
    mySLAM::RgbdImagePyramid::Ptr previous_;
    mySLAM::KeyframeTrackerConfig cfg_;
  };
  // begin Keyframe::Impl function
  KeyframeTracker::Impl::Impl() :
    graph_(),
    lt_()
  {}
  void KeyframeTracker::Impl::init(const Eigen::Affine3d& initial_transformation)
  {
    initial_transformation_ = initial_transformation;
    relative_transformation_.setIdentity();
  }
  // void KeyframeTracker::Impl::update(const mySLAM::RgbdImagePyramid::Ptr& current, Eigen::Affine3d& absolute_transformation)
  // {
  //   if(!previous_)
  //   {
  //     previous_ = current;
  //     absolute_transformation = initial_transformation_;
  //     return ;
  //   }

  //   if(!lt_.getLocalMap())
  //   {
  //     lt_.initNewLocalMap(previous_, current, initial_transformation_);
  //     lt_.getCurrentPose(absolute_transformation);
  //     return ;
  //   }
  //   lt_.update(current, absolute_transformation);
  // }
  // end KeyframeTracker::Impl function


  // begin KeyframeTracker function
  KeyframeTracker::KeyframeTracker() :
    impl_()
  {}
  KeyframeTracker::~KeyframeTracker()
  {}
  void KeyframeTracker::configureTracking(const mySLAM::DenseTracker::Config& cfg)
  {
    impl_->graph_.configureValidationTracking(cfg);
    impl_->lt_.configure(cfg);
  }
  void KeyframeTracker::init( )
  {
    init(Eigen::Affine3d::Identity());
  }

  void KeyframeTracker::init(const Eigen::Affine3d& initial_transformation)
  {
    impl_->init(initial_transformation);
  }

  // void KeyframeTracker::update(const mySLAM::RgbdImagePyramid::Ptr& current, Eigen::Affine3d& absolute_transformation)
  // {
  //   impl_->update(current, absolute_transformation);
  // }

  // end KeyframeTracker function
}  // end namespace mySLAM
