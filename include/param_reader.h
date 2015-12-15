#ifndef _PARAM_READER_H
#define _PARAM_READER_H

#include <const_param.h>
#include <string>
#include <sstream>
#include <iostream>

class ParamReader
{
public:
  ParamReader(const std::string& para_file);
  ~ParamReader(){}

  std::string getPara(const std::string& para_name);

protected:
  std::string num2string(double d)
  {
    ss.str("");
    ss.clear();
    ss << d;
    return ss.str();
  }

  std::string num2string(int i)
  {
    ss.str("");
    ss.clear();
    ss << i;
    return ss.str();
  }

  std::stringstream ss;
  std::string dataset;

  double camera_fx, camera_fy, camera_cx, camera_cy, camera_factor;

  int FirstLevel, LastLevel, MaxIterationsPerLevel;
  double Precision;
  double UseInitialEstimate, UseWeighting;
  double ScaleEstimatorType, ScaleEstimatorParam;
  double InfluenceFuntionType, InfluenceFuntionParam;
  double Mu;
  double IntensityDerivativeThreshold, DepthDerivativeThreshold;


  double MaxTranslationalDistance, MaxRotationalDistance;
  double MinEntropyRatio, MinEquationSystemConstraintRatio1;
  double UseMultiThreading1;

  double MinConstraintDistance;
  double OptimizationIterations; // might be integer;
  double OptimizationUseDenseGraph;
  double OptimizationRemoveOutliers;
  double OptimizationOutlierWeightThreshold;
  double FinalOptimizationRemoveOutliers;
  double FinalOptimizationOutlierWeightThreshold;
  double FinalOptimizationIterations;
  double FinalOptimizationUseDenseGraph;
  double NewConstraintSearchRadius;
  double NewConstraintMinEntropyRatioCoarse;
  double NewConstraintMinEntropyRatioFine;
  double UseRobustKernel;
  double MinEquationSystemConstraintRatio2;
  double UseMultiThreading2;

};

extern ParamReader* g_pParamReader;


#endif
