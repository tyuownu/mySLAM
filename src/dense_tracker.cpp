#include <iomanip>
#include <assert.h>
//#include <sophus/se3.hpp>

#include <dense_tracker.h>

#include <data_types.h>

namespace mySLAM
{
  // DenseTracker::Config
  DenseTracker::Config::Config() :
    FirstLevel(3),
    LastLevel(1),
    MaxIterationsPerLevel(100),
    Precision(5e-7),
    UseInitialEstimate(false),
    UseWeighting(true),
    Mu(0),
    // TODO: Distribution
    IntensityDerivativeThreshold(0.0f),
    DepthDerivativeThreshold(0.0f)
  {}

  size_t DenseTracker::Config::getNumLevels() const
  {
    return FirstLevel + 1;
  }

  bool DenseTracker::Config::UseEstimateSmoothing() const
  {
    return Mu > 1e-6;
  }

  bool DenseTracker::Config::IsSane() const
  {
    return FirstLevel >= LastLevel;
  }
  // end DenseTracker::Config function

  // begin DenseTracker function
  const DenseTracker::Config& DenseTracker::getDefaultConfig()
  {
    static Config defaultConfig;
    return defaultConfig;
  }

  void DenseTracker::configure(const Config& config)
  {
    assert(config.IsSane());
  }
  // end DenseTracker function

}
