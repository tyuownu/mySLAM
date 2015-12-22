#include "keyframe_graph.h"
#include <local_tracker.h>



#include <tbb/concurrent_queue.h>
#include <tbb/tbb_thread.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_reduce.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/mutex.h>

#include <boost/bind.hpp>
#include <boost/signals2.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

// #include <g2o/core/sparse_optimizer.h>
// #include <g2o/solvers/dense/linear_solver_dense.h>
// #include <g2o/solvers/pcg/linear_solver_pcg.h>
// #include <g2o/solvers/csparse/linear_solver_csparse.h>
// #include <g2o/core/block_solver.h>
// #include <g2o/core/solver.h>
// #include <g2o/core/optimization_algorithm_levenberg.h>
// #include <g2o/core/optimization_algorithm_gauss_newton.h>
// #include <g2o/core/optimization_algorithm_dogleg.h>
// #include <g2o/core/robust_kernel_impl.h>
// #include <g2o/core/estimate_propagator.h>
// #include <g2o/types/slam3d/vertex_se3.h>
// #include <g2o/types/slam3d/edge_se3.h>
// #include <g2o/types/slam3d/edge_se3_offset.h>

namespace mySLAM
{
  class KeyframeGraphImpl
  {
  public:
    friend class KeyframeGraph;
    KeyframeGraphImpl();
    ~KeyframeGraphImpl();

    void configure(const mySLAM::KeyframeGraphConfig& cfg);
    void configureValidationTracking(const DenseTracker::Config& cfg);

  private:
    // typedef g2o::BlockSolver_6_3 BlockSolver;
    // typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;
    // typedef tbb::enumerable_thread_specific<ConstraintProposalValidatorPtr> ConstraintProposalValidatorPool;

    KeyframePtr active_;

    DenseTracker::Config validation_tracker_cfg_, constraint_tracker_cfg_;
    KeyframeGraphConfig cfg_;
    KeyframeGraph* me_;
  };
  // begin KeyframeGraphImpl function
  KeyframeGraphImpl::KeyframeGraphImpl()
  {}

  KeyframeGraphImpl::~KeyframeGraphImpl()
  {}

  void KeyframeGraphImpl::configure(const mySLAM::KeyframeGraphConfig& cfg)
  {
    cfg_ = cfg;
    // constraint_search_.reset(new NearestNeighborConstraintSearch(cfg_.NewConstraintSearchRadius));
  }

  void KeyframeGraphImpl::configureValidationTracking(const DenseTracker::Config& cfg)
  {
    constraint_tracker_cfg_                              = DenseTracker::getDefaultConfig();
    constraint_tracker_cfg_.FirstLevel                   = 3;
    constraint_tracker_cfg_.LastLevel                    = 1;
    constraint_tracker_cfg_.Precision                    = cfg.Precision;
    constraint_tracker_cfg_.UseInitialEstimate           = true;
    constraint_tracker_cfg_.Mu                           = cfg.Mu;
    constraint_tracker_cfg_.IntensityDerivativeThreshold = cfg.IntensityDerivativeThreshold;
    constraint_tracker_cfg_.DepthDerivativeThreshold     = cfg.DepthDerivativeThreshold;

    validation_tracker_cfg_                              = DenseTracker::getDefaultConfig();
    validation_tracker_cfg_.FirstLevel                   = 3;
    validation_tracker_cfg_.LastLevel                    = 3;
    validation_tracker_cfg_.Precision                    = cfg.Precision;
    validation_tracker_cfg_.UseInitialEstimate           = true;
    validation_tracker_cfg_.Mu                           = cfg.Mu;
    validation_tracker_cfg_.IntensityDerivativeThreshold = cfg.IntensityDerivativeThreshold;
    validation_tracker_cfg_.DepthDerivativeThreshold     = cfg.DepthDerivativeThreshold;
  }
  // end KeyframeGraphImpl function

  // begin KeyframeGraph function
  KeyframeGraph::KeyframeGraph() :
    impl_(new KeyframeGraphImpl())
  {}

  KeyframeGraph::~KeyframeGraph()
  {}
  void KeyframeGraph::configure(const mySLAM::KeyframeGraphConfig& config)
  {
    impl_->configure(config);
  }
  void KeyframeGraph::configureValidationTracking(const mySLAM::DenseTracker::Config& cfg)
  {
    impl_->configureValidationTracking(cfg);
  }
  // end KeyframeGraph function

}  // end namespace mySLAM
