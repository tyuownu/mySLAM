#include <local_tracker.h>


#include <tbb/parallel_invoke.h>
#include <tbb/tbb_thread.h>

namespace mySLAM
{
  // typedef boost::shared_ptr<mySLAM::PointSelection> PointSelectionPtr;
  typedef boost::shared_ptr<mySLAM::DenseTracker> DenseTrackerPtr;

  struct LocalTrackerImpl
  {
    friend class LocalTracker;
    DenseTrackerPtr keyframe_tracker_, odometry_tracker_;
    // mySLAM::ValidPointAndGradientThresholdPredicate predicate;

    mySLAM::AffineTransformd last_keyframe_pose_;
    // PointSelectionPtr keyframe_points_, active_frame_points_;
    bool force_;

    // LocalTracker::AcceptSignal accept_;
    // LocalTracker::MapInitializedSignal map_initialized_;
    // LocalTracker::MapCompleteSignal map_complete_;
  };

  void LocalTracker::configure(const DenseTracker::Config& config)
  {
    impl_->keyframe_tracker_->configure(config);
    impl_->odometry_tracker_->configure(config);
  }
}  // end namespace mySLAM
