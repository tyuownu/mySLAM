#ifndef _KEYFRAME_GRAPH_H
#define _KEYFRAME_GRAPH_H

#include <local_map.h>
#include <keyframe.h>
#include <keyframe_config.h>

#include <dense_tracker.h>
#include <boost/function.hpp>

namespace mySLAM
{
  class KeyframeGraphImpl;
  typedef boost::scoped_ptr<KeyframeGraphImpl> KeyframeGraphImplPtr;

  class KeyframeGraph
  {
  public:
    typedef void MapChangedCallbackSignature(KeyframeGraph& graph);
    typedef boost::function<MapChangedCallbackSignature> MapChangedCallback;

    KeyframeGraph();
    virtual ~KeyframeGraph();

    const mySLAM::KeyframeGraphConfig& configuration() const;
    void configure(const mySLAM::KeyframeGraphConfig& config);
    void configureValidationTracking(const mySLAM::DenseTracker::Config& cfg);
    void add(const mySLAM::LocalMap::Ptr& keyframe);
    void finalOptimization();
    void addMapChangedCallback(const mySLAM::KeyframeGraph::MapChangedCallback& callback);
    const mySLAM::KeyframeVector& keyframes() const;
    /* const g2o::SparseOptimizer& graph() const; */
    cv::Mat computeIntensityErrorImage(int edge_id, bool use_measurement = true) const;
    void debugLoopClosureConstraint(int keyframe1, int keyframe2) const;

  private:
    KeyframeGraphImplPtr impl_;
  };

}  // end namespace mySLAM





#endif
