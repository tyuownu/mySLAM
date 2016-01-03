#include "local_map.h"
#include <tracking_result_evaluation.h>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/solver.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3_offset.h>

namespace mySLAM
{
  // begin some basic function
  Eigen::Isometry3d toIsometry(const Eigen::Affine3d& pose)
  {
    Eigen::Isometry3d p(pose.rotation());
    p.translation() = pose.translation();
    return p;
  }
  static Eigen::Affine3d toAffine(const Eigen::Isometry3d& pose)
  {
    Eigen::Affine3d p(pose.rotation());
    p.translation() = pose.translation();

    return p;
  }
  // end some basic function
  struct LocalMapImpl
  {
    typedef g2o::BlockSolver_6_3 BlockSolver;
    typedef g2o::LinearSolverCSparse<BlockSolver::PoseMatrixType> LinearSolver;
    mySLAM::RgbdImagePyramid::Ptr keyframe_, current_;
    g2o::VertexSE3 *keyframe_vertex_, *previous_vertex_, *current_vertex_;
    g2o::SparseOptimizer graph_;
    int max_vertex_id_, max_edge_id_;
    mySLAM::TrackingResultEvaluation::ConstPtr evaluation_;
    LocalMapImpl(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                 const mySLAM::AffineTransformd& keyframe_pose);
    g2o::VertexSE3* addFrameVertex();
  };

  // begin LocalMapImpl function
   LocalMapImpl::LocalMapImpl(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                              const mySLAM::AffineTransformd& keyframe_pose) :
     keyframe_(keyframe),
     keyframe_vertex_(0),
     previous_vertex_(0),
     current_vertex_(0),
     max_vertex_id_(1),
     max_edge_id_(1)
  {
    graph_.setAlgorithm(
      new g2o::OptimizationAlgorithmLevenberg(
        new BlockSolver(
          new LinearSolver()
          )
        )
      );
    graph_.setVerbose(false);

    keyframe_vertex_ = addFrameVertex();
    keyframe_vertex_->setFixed(true);
    keyframe_vertex_->setEstimate(toIsometry(keyframe_pose));
  }

  g2o::VertexSE3* LocalMapImpl::addFrameVertex()
  {
    g2o::VertexSE3* frame_vertex = new g2o::VertexSE3();
    frame_vertex->setId(max_vertex_id_++);
    // frame_vertex->setUserData(new g2o::OptimizableGraph::Data());

    graph_.addVertex(frame_vertex);
    return frame_vertex;
  }
  // end LocalMapImpl function

  // begin LocalMap function

  LocalMap::LocalMap(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                     const mySLAM::AffineTransformd& keyframe_pose) :
    impl_(new LocalMapImpl(keyframe, keyframe_pose))
  {}

  LocalMap::~LocalMap()
  {}

  LocalMap::Ptr LocalMap::create(const mySLAM::RgbdImagePyramid::Ptr& keyframe,
                                 const mySLAM::AffineTransformd& keyframe_pose)
  {
    LocalMap::Ptr result(new LocalMap(keyframe, keyframe_pose));
    return result;
  }

  void LocalMap::getCurrentFramePose(mySLAM::AffineTransformd& current_pose)
  {
    current_pose = getCurrentFramePose();
  }

  mySLAM::AffineTransformd  LocalMap::getCurrentFramePose()
  {
    return toAffine(impl_->current_vertex_->estimate());
  }
  void LocalMap::addFrame(const mySLAM::RgbdImagePyramid::Ptr& frame)
  {
    impl_->current_ = frame;
    impl_->previous_vertex_ = impl_->current_vertex_;
    // impl_->current_vertex_ = impl_->addFrameVertex(g2o::OptimizableGraph::Data());
  }

  void LocalMap::addKeyframeMeasurement(const mySLAM::AffineTransformd& pose, const mySLAM::Matrix6d& information)
  {
    // impl_->addTransformationEdge(impl_->keyframe_vertex_, impl_->current_vertex_, pose, information);
  }
  // end LocalMap function
}  // end namespace mySLAM
