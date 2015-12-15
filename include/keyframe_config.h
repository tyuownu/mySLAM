#ifndef _KEYFRAME_CONFIG_H
#define _KEYFRAME_CONFIG_H

#include <ostream>
#include <limits>

namespace mySLAM
{
  struct KeyframeTrackerConfig
  {
    bool UseMultiThreading1;
    double MaxTranslationalDistance;
    double MaxRotationalDistance;
    double MinEntropyRatio;
    double MinEquationSystemConstraintRatio1;

    KeyframeTrackerConfig() :
      UseMultiThreading1(true),
      MaxTranslationalDistance(0.2),
      MaxRotationalDistance(std::numeric_limits<double>::max()),
      MinEntropyRatio(0.91),
      MinEquationSystemConstraintRatio1(0.33)
      {}
  };

  struct KeyframeGraphConfig
  {
    bool UseRobustKernel;
    bool UseMultiThreading2;

    double NewConstraintSearchRadius;
    double NewConstraintMinEntropyRatioCoarse;
    double NewConstraintMinEntropyRatioFine;
    double MinEquationSystemConstraintRatio2;

    bool OptimizationUseDenseGraph;
    bool OptimizationRemoveOutliers;
    double OptimizationOutlierWeightThreshold;
    size_t OptimizationIterations;

    bool FinalOptimizationUseDenseGraph;
    bool FinalOptimizationRemoveOutliers;
    double FinalOptimizationOutlierWeightThreshold;
    size_t FinalOptimizationIterations;

    size_t MinConstraintDistance;
    KeyframeGraphConfig() :
      UseRobustKernel(true),
      UseMultiThreading2(true),
      NewConstraintSearchRadius(1.0),
      NewConstraintMinEntropyRatioCoarse(0.7),
      NewConstraintMinEntropyRatioFine(0.9),
      MinEquationSystemConstraintRatio2(0.2),
      MinConstraintDistance(0),
      OptimizationUseDenseGraph(false),
      OptimizationIterations(20),
      OptimizationRemoveOutliers(false),
      OptimizationOutlierWeightThreshold(0.0),
      FinalOptimizationUseDenseGraph(true),
      FinalOptimizationIterations(5000),
      FinalOptimizationRemoveOutliers(false),
      FinalOptimizationOutlierWeightThreshold(0.0)
      {}
  };
}  // end namespace mySLAM

#endif
