#include <local_tracker.h>
#include <point_selection.h>


#include <tbb/parallel_invoke.h>
#include <tbb/tbb_thread.h>

namespace mySLAM
{
  struct LocalTrackerImpl
  {
    friend class LocalTracker;
    DenseTrackerPtr keyframe_tracker_, odometry_tracker_;
    mySLAM::ValidPointAndGradientThresholdPredicate predicate;
    mySLAM::AffineTransformd last_keyframe_pose_;
    PointSelectionPtr keyframe_points_, active_frame_points_;

    bool force_;

    LocalTracker::AcceptSignal accept_;
    LocalTracker::MapInitializedSignal map_initialized_;
    LocalTracker::MapCompleteSignal map_complete_;
    static void match(const DenseTrackerPtr& tracker,
                      const PointSelectionPtr& ref,
                      const mySLAM::RgbdImagePyramid::Ptr& cur,
                      LocalTracker::TrackingResult* r)
    {
      tracker->match(*ref, *cur, *r);
    }
  };

  // begin LocalTrackerImpl function
  // end LocalTrackerImpl function


  // begin LocalTracker function
  LocalTracker::LocalTracker() :
    impl_(new LocalTrackerImpl())
  {
    impl_->keyframe_tracker_.reset(new mySLAM::DenseTracker());
    impl_->odometry_tracker_.reset(new mySLAM::DenseTracker());
    impl_->last_keyframe_pose_.setIdentity();
    impl_->force_ = false;
    impl_->keyframe_points_.reset(new mySLAM::PointSelection(impl_->predicate));
    impl_->active_frame_points_.reset(new mySLAM::PointSelection(impl_->predicate));
  }
  LocalTracker::~LocalTracker()
  {}

  mySLAM::LocalMap::Ptr LocalTracker::getLocalMap() const
  {
    return local_map_;
  }

  void LocalTracker::getCurrentPose(mySLAM::AffineTransformd& pose)
  {
    local_map_->getCurrentFramePose(pose);
  }

  boost::signals2::connection LocalTracker::addMapInitializedCallback(const MapInitializedCallback& callback)
  {
    return impl_->map_initialized_.connect(callback);
  }

  const mySLAM::DenseTracker::Config& LocalTracker::configuration() const
  {
    return impl_->odometry_tracker_->configuration();
  }

  void LocalTracker::configure(const DenseTracker::Config& config)
  {
    impl_->keyframe_tracker_->configure(config);
    impl_->odometry_tracker_->configure(config);
  }

  void LocalTracker::initNewLocalMap(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                                     const mySLAM::RgbdImagePyramid::Ptr& frame,
                                     const mySLAM::AffineTransformd& keyframe_pose)
  {
    impl_->keyframe_points_->setRgbdImagePyramid(*keyframe);
    impl_->active_frame_points_->setRgbdImagePyramid(*frame);

    TrackingResult r_odometry;
    r_odometry.Transformation.setIdentity();

    impl_->odometry_tracker_->match(*(impl_->keyframe_points_), *frame, r_odometry);
    impl_->last_keyframe_pose_ = r_odometry.Transformation;

    initNewLocalMap(keyframe, frame, r_odometry, keyframe_pose);
  }

  void LocalTracker::initNewLocalMap(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                                     const mySLAM::RgbdImagePyramid::Ptr& frame,
                                     TrackingResult& r_odometry,
                                     const mySLAM::AffineTransformd& keyframe_pose)
  {
    if(r_odometry.isNaN())
    {
      std::cout<<"NaN in Map Initialization!"<<std::endl;
      r_odometry.setIdentity();
    }

    local_map_ = LocalMap::create(keyframe, keyframe_pose);
    local_map_->addFrame(frame);
    local_map_->addKeyframeMeasurement(r_odometry.Transformation, r_odometry.Information);
    impl_->map_initialized_(*this, local_map_, r_odometry);
  }

  void LocalTracker::update(const mySLAM::RgbdImagePyramid::Ptr& image, mySLAM::AffineTransformd& pose)
  {
    const mySLAM::DenseTracker::Config& config = impl_->keyframe_tracker_->configuration();
    image->build(config.getNumLevels());

    for(int id = config.LastLevel; id <= config.FirstLevel; ++id)
    {
      image->level(id).buildPointCloud();
      image->level(id).buildAccelerationStructure();
    }

    TrackingResult r_odometry, r_keyframe;
    r_odometry.Transformation.setIdentity();
    r_keyframe.Transformation = impl_->last_keyframe_pose_.inverse(Eigen::Isometry);
  }

  // end LocalTracker function
}  // end namespace mySLAM
