#ifndef _KEYFRAME_H
#define _KEYFRAME_H

#include <rgbd_image.h>
#include <tracking_result_evaluation.h>

#include <Eigen/Geometry>
#include <vector>

namespace mySLAM
{
  class Keyframe
  {
  public:
    Keyframe() : id_(-1) {}
    virtual ~Keyframe() {}

  protected:
    short id_;
    mySLAM::RgbdImagePyramid::Ptr image_;
    Eigen::Affine3d pose_;
    mySLAM::TrackingResultEvaluation::ConstPtr evaluation_;
  };

  typedef boost::shared_ptr<Keyframe> KeyframePtr;
  typedef std::vector<KeyframePtr> KeyframeVector;

}  // end namespace mySLAM


#endif
