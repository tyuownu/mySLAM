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
  // end DenseTracker::Config
  const DenseTracker::Config& DenseTracker::getDefaultConfig()
  {
    static Config defaultConfig;
    return defaultConfig;
  }

}
