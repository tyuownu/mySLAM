#ifndef _KEYFRAME_TRACKER_H
#define _KEYFRAME_TRACKER_H

#include <rgbd_image.h>
#include <dense_tracker.h>
#include <keyframe_config.h>
#include <keyframe_graph.h>
#include <local_tracker.h>
namespace mySLAM
{
  class KeyframeTracker
  {
  public:
    KeyframeTracker();
    ~KeyframeTracker();

    const mySLAM::DenseTracker::Config& trackingConfiguration() const;
    const mySLAM::KeyframeTrackerConfig& keyframeSelectionConfiguration() const;
    const mySLAM::KeyframeGraphConfig& mappingConfiguration() const;

    void configureTracking(const mySLAM::DenseTracker::Config& cfg);
    void configureKeyframeSelection(const mySLAM::KeyframeTrackerConfig& cfg);
    void configureMapping(const mySLAM::KeyframeGraphConfig& cfg);

    void init();
    void init(const Eigen::Affine3d& initial_transformation);
    void update(const mySLAM::RgbdImagePyramid::Ptr& current, Eigen::Affine3d& absolute_transformation);

    void forceKeyframe();
    void finish();
  private:
    class Impl;
    boost::shared_ptr<Impl> impl_;
  };

}  // end namespace mySLAM

#endif
