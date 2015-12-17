#ifndef _LOCAL_MAP_H
#define _LOCAL_MAP_H

#include <rgbd_image.h>
#include <g2o/core/sparse_optimizer.h>

namespace mySLAM
{
  struct LocalMapImpl;
  class LocalMap
  {
  public:
    LocalMap();
    virtual ~LocalMap();

    typedef boost::shared_ptr<LocalMap> Ptr;
    typedef boost::shared_ptr<const LocalMap> ConstPtr;

    static LocalMap::Ptr create(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                                const mySLAM::AffineTransformd& keyframe_pose);
    mySLAM::RgbdImagePyramid::Ptr getKeyframe();
    void setKeyframePose(const mySLAM::AffineTransformd& keyframe_pose);
    mySLAM::RgbdImagePyramid::Ptr getCurrentFrame();
    void getCurrentFramePose(mySLAM::AffineTransformd& current_pose);
    mySLAM::AffineTransformd getCurrentFramePose();

    g2o::SparseOptimizer& getGraph();
    void setEvaluation(mySLAM::TrackingResultEvaluation::ConstPtr& evaluation);
    mySLAM::TrackingResultEvaluation::ConstPtr getEvaluation();
    void addFrame(const mySLAM::RgbdImagePyramid::Ptr& frame);
    void addOdometryMeasurement(const mySLAM::AffineTransformd& pose, const mySLAM::Matrix6d& information);
    void addKeyframeMeasurement(const mySLAM::AffineTransformd& pose, const mySLAM::Matrix6d& information);
    void optimize();
  private:
    LocalMap(const mySLAM::RgbdImagePyramid::Ptr& keyframe, const mySLAM::AffineTransformd& keyframe_pose);
    boost::scoped_ptr<LocalMapImpl> impl_;
  };

}  // end namespace mySLAM

#endif
