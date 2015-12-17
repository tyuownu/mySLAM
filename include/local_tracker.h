#ifndef _LOCAL_TRACKER_H
#define _LOCAL_TRACKER_H

#include <rgbd_image.h>
#include <local_map.h>

#include <boost/scoped_ptr.hpp>
#include <boost/signals2.hpp>

namespace mySLAM
{
  struct LocalTrackerImpl;

  class LocalTracker
  {
  public:
    LocalTracker();
    virtual ~LocalTracker();
    typedef ::mySLAM::DenseTracker::Result TrackingResult;

    mySLAM::LocalMap::Ptr getLocalMap() const;
    void getCurrentPose(mySLAM::AffineTransformd& pose);
    void initNewLocalMap(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                         const mySLAM::RgbdImagePyramid::Ptr& frame,
                         const mySLAM::AffineTransformd& keyframe_pose = mySLAM::AffineTransformd::Identity());
    const mySLAM::DenseTracker::Config& configuration() const;
    void configure(const mySLAM::DenseTracker::Config& config);
    void update(const mySLAM::RgbdImagePyramid::Ptr& image, mySLAM::AffineTransformd& pose);
    void forceCompleteCurrentLocalMap();

  private:
    boost::scoped_ptr<LocalTrackerImpl> impl_;
    mySLAM::LocalMap::Ptr local_map_;
    void initNewLocalMap(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                         const mySLAM::RgbdImagePyramid::Ptr& frame,
                         TrackingResult& r_odometry,
                         const mySLAM::AffineTransformd& keyframe_pose);
  };

  struct LocalTrackerImpl
  {
    friend class LocalTracker;
    DenseTrackerPtr keyframe_tracker_, odometry_tracker_;
    mySLAM::ValidPointAndGradientThresholdPredicate predicate;
    mySLAM::AffineTransformd last_keyframe_pose_;
    PointSelectionPtr keyframe_points_, active_frame_points_;

    bool force_;
    static void match(const DenseTrackerPtr& tracker,
                      const PointSelectionPtr& ref,
                      const mySLAM::RgbdImagePyramid::Ptr& cur,
                      LocalTracker::TrackingResult* r)
    {
      tracker->match(*ref, *cur, *r);
    }
  };

}  // end namespace mySLAM

#endif
