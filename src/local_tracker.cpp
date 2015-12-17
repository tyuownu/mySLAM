#include <local_tracker.h>


#include <tbb/parallel_invoke.h>
#include <tbb/tbb_thread.h>

namespace mySLAM
{


  LocalTracker::LocalTracker()
  {}
  LocalTracker::~LocalTracker()
  {}
  void LocalTracker::configure(const DenseTracker::Config& config)
  {
    impl_->keyframe_tracker_->configure(config);
    impl_->odometry_tracker_->configure(config);
  }
}  // end namespace mySLAM
