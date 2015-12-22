#ifndef _UPDATE_CFG_H
#define _UPDATE_CFG_H

#include <dense_tracker.h>
#include <param_reader.h>
#include <const_param.h>
#include <keyframe_config.h>
/**
 * update cfg.
 */
bool updateDenseTrackerConfig(mySLAM::DenseTracker::Config cfg)
{
  cfg.FirstLevel                   = atoi( g_pParamReader->getPara("FirstLevel").c_str() );
  cfg.LastLevel                    = atoi( g_pParamReader->getPara("LastLevel").c_str() );
  cfg.MaxIterationsPerLevel        = atoi( g_pParamReader->getPara("MaxIterationsPerLevel").c_str() );
  cfg.Precision                    = atof( g_pParamReader->getPara("Precision").c_str() );
  cfg.Mu                           = atof( g_pParamReader->getPara("Mu").c_str() );
  cfg.UseInitialEstimate           = ( g_pParamReader->getPara("UseInitialEstimate") == std::string("yes") )?true:false;
  cfg.UseWeighting                 = ( g_pParamReader->getPara("UseWeighting") == std::string("yes") )?true:false;
  cfg.IntensityDerivativeThreshold = atof( g_pParamReader->getPara("IntensityDerivativeThreshold").c_str() );
  cfg.DepthDerivativeThreshold     = atof( g_pParamReader->getPara("DepthDerivativeThreshold").c_str() );

  std::cout<<cfg.FirstLevel<<std::endl
           <<cfg.LastLevel<<std::endl
           <<cfg.MaxIterationsPerLevel<<std::endl
           <<cfg.Precision<<std::endl
           <<cfg.Mu<<std::endl
           <<cfg.UseInitialEstimate<<std::endl
           <<cfg.UseWeighting<<std::endl
           <<cfg.IntensityDerivativeThreshold<<std::endl
           <<cfg.DepthDerivativeThreshold<<std::endl;
}

bool updateKeyframeConfig(mySLAM::KeyframeTrackerConfig frontend_cfg,
                          mySLAM::KeyframeGraphConfig backend_cfg)
{
  // frontend
  frontend_cfg.UseMultiThreading1                = ( g_pParamReader->getPara("UseMultiThreading1") == std::string("yes") )?true:false;
  frontend_cfg.MaxTranslationalDistance          = atof( g_pParamReader->getPara("MaxTranslationalDistance").c_str() );
  frontend_cfg.MaxRotationalDistance             = atof( g_pParamReader->getPara("MaxRotationalDistance").c_str() );
  frontend_cfg.MinEntropyRatio                   = atof( g_pParamReader->getPara("MinEntropyRatio").c_str() );
  frontend_cfg.MinEquationSystemConstraintRatio1 = atof( g_pParamReader->getPara("MinEquationSystemConstraintRatio1").c_str() );

  // backend
  backend_cfg.UseRobustKernel                         = ( g_pParamReader->getPara("UseRobustKernel") == std::string("yes") )?true:false;
  backend_cfg.UseMultiThreading2                      = ( g_pParamReader->getPara("UseMultiThreading2") == std::string("yes") )?true:false;
  backend_cfg.NewConstraintSearchRadius               = atof( g_pParamReader->getPara("NewConstraintSearchRadius").c_str() );
  backend_cfg.NewConstraintMinEntropyRatioCoarse      = atof( g_pParamReader->getPara("NewConstraintMinEntropyRatioCoarse").c_str() );
  backend_cfg.NewConstraintMinEntropyRatioFine        = atof( g_pParamReader->getPara("NewConstraintMinEntropyRatioFine").c_str() );
  backend_cfg.MinEquationSystemConstraintRatio2       = atof( g_pParamReader->getPara("MinEquationSystemConstraintRatio2").c_str() );
  backend_cfg.OptimizationUseDenseGraph               = ( g_pParamReader->getPara("OptimizationUseDenseGraph") == std::string("yes") )?true:false;
  backend_cfg.OptimizationRemoveOutliers              = ( g_pParamReader->getPara("OptimizationRemoveOutliers") == std::string("yes") )?true:false;
  backend_cfg.OptimizationOutlierWeightThreshold      = atof( g_pParamReader->getPara("OptimizationOutlierWeightThreshold").c_str() );
  backend_cfg.OptimizationIterations                  = atoi( g_pParamReader->getPara("OptimizationIterations").c_str() );
  backend_cfg.FinalOptimizationUseDenseGraph          = ( g_pParamReader->getPara("FinalOptimizationUseDenseGraph") == std::string("yes") )?true:false;
  backend_cfg.FinalOptimizationRemoveOutliers         = ( g_pParamReader->getPara("FinalOptimizationRemoveOutliers") == std::string("yes") );
  backend_cfg.FinalOptimizationOutlierWeightThreshold = atof( g_pParamReader->getPara("FinalOptimizationOutlierWeightThreshold").c_str() );
  backend_cfg.FinalOptimizationIterations             = atoi( g_pParamReader->getPara("FinalOptimizationIterations").c_str() );
  backend_cfg.MinConstraintDistance                   = atoi( g_pParamReader->getPara("MinConstraintDistance").c_str() );
}


#endif
