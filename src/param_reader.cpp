#include "param_reader.h"
#include <fstream>
#include "yaml-cpp/yaml.h"
#include "const_param.h"

ParamReader* g_pParamReader = new ParamReader(parameter_file_address);
ParamReader::ParamReader(const std::string& para_file)
{
  std::cout<<"init ParamReader, file addr = "<<para_file<<std::endl;
  std::ifstream fin(para_file.c_str());
  if(!fin) std::cout<<"error happens!"<<std::endl;

  YAML::Parser parser(fin);
  YAML::Node config;
  try
  {
    parser.GetNextDocument(config);
  }
  catch (YAML::ParserException& e)
  {
    std::cerr<<e.what()<<"\n";
    return ;
  }

  // read message
  config[ "dataset" ] >> dataset;


  // camera setting
  config[ "camera_fx" ]     >> camera_fx;
  config[ "camera_fy" ]     >> camera_fy;
  config[ "camera_cx" ]     >> camera_cx;
  config[ "camera_cy" ]     >> camera_cy;
  config[ "camera_factor" ] >> camera_factor;

  // DenseTracker::Config
  config[ "FirstLevel" ]                   >> FirstLevel;
  config[ "LastLevel" ]                    >> LastLevel;
  config[ "MaxIterationsPerLevel" ]        >> MaxIterationsPerLevel;
  config[ "Precision" ]                    >> Precision;
  config[ "UseInitialEstimate" ]           >> UseInitialEstimate;
  config[ "UseWeighting" ]                 >> UseWeighting;
  config[ "ScaleEstimatorType" ]           >> ScaleEstimatorType;
  config[ "ScaleEstimatorParam" ]          >> ScaleEstimatorParam;
  config[ "InfluenceFuntionType" ]         >> InfluenceFuntionType;
  config[ "InfluenceFuntionParam" ]        >> InfluenceFuntionParam;
  config[ "Mu" ]                           >> Mu;
  config[ "IntensityDerivativeThreshold" ] >> IntensityDerivativeThreshold;
  config[ "DepthDerivativeThreshold" ]     >> DepthDerivativeThreshold;

  // KeyframeSlamConfig
  // frontend
  config[ "MaxTranslationalDistance" ]          >> MaxTranslationalDistance;
  config[ "MaxRotationalDistance" ]             >> MaxRotationalDistance;
  config[ "MinEntropyRatio" ]                   >> MinEntropyRatio;
  config[ "MinEquationSystemConstraintRatio1" ] >> MinEquationSystemConstraintRatio1;
  config[ "UseMultiThreading1" ]                >> UseMultiThreading1;
  //backend
  config[ "MinConstraintDistance" ]                   >> MinConstraintDistance;
  config[ "OptimizationIterations" ]                  >> OptimizationIterations;
  config[ "OptimizationUseDenseGraph" ]               >> OptimizationUseDenseGraph;
  config[ "OptimizationRemoveOutliers" ]              >> OptimizationRemoveOutliers;
  config[ "OptimizationOutlierWeightThreshold" ]      >> OptimizationOutlierWeightThreshold;
  config[ "FinalOptimizationRemoveOutliers" ]         >> FinalOptimizationRemoveOutliers;
  config[ "FinalOptimizationOutlierWeightThreshold" ] >> FinalOptimizationOutlierWeightThreshold;
  config[ "FinalOptimizationIterations" ]             >> FinalOptimizationIterations;
  config[ "FinalOptimizationUseDenseGraph" ]          >> FinalOptimizationUseDenseGraph;
  config[ "NewConstraintSearchRadius" ]               >> NewConstraintSearchRadius;
  config[ "NewConstraintMinEntropyRatioCoarse" ]      >> NewConstraintMinEntropyRatioCoarse;
  config[ "NewConstraintMinEntropyRatioFine" ]        >> NewConstraintMinEntropyRatioFine;
  config[ "UseRobustKernel" ]                         >> UseRobustKernel;
  config[ "MinEquationSystemConstraintRatio2" ]       >> MinEquationSystemConstraintRatio2;
  config[ "UseMultiThreading2" ]                      >> UseMultiThreading2;

}

std::string ParamReader::getPara( const std::string& para_name )
{
  if ( para_name == std::string("dataset") )
    return dataset;

  if ( para_name == std::string("camera_fx") )
    return num2string(camera_fx);
  if ( para_name == std::string("camera_fy") )
    return num2string(camera_fy);
  if ( para_name == std::string("camera_cx") )
    return num2string(camera_cx);
  if ( para_name == std::string("camera_cy") )
    return num2string(camera_cy);
  if ( para_name == std::string("camera_factor") )
    return num2string(camera_factor);

  if ( para_name == std::string("FirstLevel") )
    return num2string(FirstLevel);
  if ( para_name == std::string("LastLevel") )
    return num2string(LastLevel);
  if ( para_name == std::string("MaxIterationsPerLevel") )
    return num2string(MaxIterationsPerLevel);
  if ( para_name == std::string("Precision") )
    return num2string(Precision);
  if ( para_name == std::string("UseInitialEstimate") )
    return num2string(UseInitialEstimate);
  if ( para_name == std::string("UseWeighting") )
    return num2string(UseWeighting);
  if ( para_name == std::string("ScaleEstimatorType") )
    return num2string(ScaleEstimatorType);
  if ( para_name == std::string("ScaleEstimatorParam") )
    return num2string(ScaleEstimatorParam);
  if ( para_name == std::string("InfluenceFuntionType") )
    return num2string(InfluenceFuntionType);
  if ( para_name == std::string("Mu") )
    return num2string(Mu);
  if ( para_name == std::string("IntensityDerivativeThreshold") )
    return num2string(IntensityDerivativeThreshold);
  if ( para_name == std::string("DepthDerivativeThreshold") )
    return num2string(DepthDerivativeThreshold);
  // KeyframeSlamConfig
  // frontend
  if ( para_name == std::string("MaxTranslationalDistance") )
    return num2string(MaxTranslationalDistance);
  if ( para_name == std::string("MaxRotationalDistance") )
    return num2string(MaxRotationalDistance);
  if ( para_name == std::string("MinEntropyRatio") )
    return num2string(MinEntropyRatio);
  if ( para_name == std::string("MinEquationSystemConstraintRatio1") )
    return num2string(MinEquationSystemConstraintRatio1);
  if ( para_name == std::string("UseMultiThreading1") )
    return num2string(UseMultiThreading1);

  // backend
  if ( para_name == std::string("MinConstraintDistance") )
    return num2string(MinConstraintDistance);
  if ( para_name == std::string("OptimizationIterations") )
    return num2string(OptimizationIterations);
  if ( para_name == std::string("OptimizationUseDenseGraph") )
    return num2string(OptimizationUseDenseGraph);
  if ( para_name == std::string("OptimizationRemoveOutliers") )
    return num2string(OptimizationRemoveOutliers);
  if ( para_name == std::string("OptimizationOutlierWeightThreshold") )
    return num2string(OptimizationOutlierWeightThreshold);
  if ( para_name == std::string("FinalOptimizationRemoveOutliers") )
    return num2string(FinalOptimizationRemoveOutliers);
  if ( para_name == std::string("FinalOptimizationOutlierWeightThreshold") )
    return num2string(FinalOptimizationOutlierWeightThreshold);
  if ( para_name == std::string("FinalOptimizationIterations") )
    return num2string(FinalOptimizationIterations);
  if ( para_name == std::string("FinalOptimizationUseDenseGraph") )
    return num2string(FinalOptimizationUseDenseGraph);
  if ( para_name == std::string("NewConstraintSearchRadius") )
    return num2string(NewConstraintSearchRadius);
  if ( para_name == std::string("NewConstraintMinEntropyRatioCoarse") )
    return num2string(NewConstraintMinEntropyRatioCoarse);
  if ( para_name == std::string("UseRobustKernel") )
    return num2string(UseRobustKernel);
  if ( para_name == std::string("MinEquationSystemConstraintRatio2") )
    return num2string(MinEquationSystemConstraintRatio2);
  if ( para_name == std::string("UseMultiThreading2") )
    return num2string(UseMultiThreading2);
}
