#ifndef _KEYFRAME_GRAPH_H
#define _KEYFRAME_GRAPH_H

#include <local_map.h>
#include <keyframe.h>

#include <dense_tracking.h>
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

    const KeyframeGraphConfig& configuration() const;
    void configure(const KeyframeGraphConfig& config);
    void configureValidationTracking(const DenseTracker::Config& cfg);
    void add(const LocalMap::Ptr& keyframe);
    void finalOptimization();
    void addMapChangedCallback(const KeyframeGraph::MapChangedCallback& callback);
    const KeyframeVector& keyframes() const;
    const g2o::SparseOptimizer& graph() const;
    cv::Mat computeIntensityErrorImage(int edge_id, bool use_measurement = true) const;
    void debugLoopClosureConstraint(int keyframe1, int keyframe2) const;

  private:
    KeyframeGraphImplPtr impl_;
  };

}  // end namespace mySLAM





#endif
