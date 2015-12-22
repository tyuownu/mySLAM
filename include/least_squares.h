#ifndef _LEAST_SQUARES_H
#define _LEAST_SQUARES_H

#include <Eigen/Core>
#include <opencv2/core/core.hpp>

#include <data_types.h>
#include <math_sse.h>

namespace mySLAM
{
  class LeastSquaresInterface
  {
  public:
    virtual ~LeastSquaresInterface();
    virtual void initialize(const size_t maxnum_constraints) = 0;
    virtual void update(const Vector6& J, const NumType& res, const NumType& weight = 1.0f) = 0;
    virtual void update(const Eigen::Matrix<NumType, 2, 6>& J, const Eigen::Matrix<NumType, 2, 1>& res, const Eigen::Matrix<NumType, 2, 2>& weight) {}
    virtual void finish() = 0;
    virtual void solve(Vector6& x) = 0;
  };

  class NormalEquationsLeastSquares : public LeastSquaresInterface
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    OptimizedSelfAdjointMatrix6x6f A_opt;
    Matrix6x6 A;
    Vector6 b;

    double error;
    size_t maxnum_constraints, num_constraints;
    virtual ~NormalEquationsLeastSquares();

    virtual void initialize(const size_t maxnum_constraints);
    virtual void update(const Vector6& J, const NumType& res, const NumType& weight = 1.0f);
    virtual void update(const Eigen::Matrix<NumType, 2, 6>& J, const Eigen::Matrix<NumType, 2, 1>& res, const Eigen::Matrix<NumType, 2, 2>& weight);
    virtual void finish();
    virtual void solve(Vector6& x);
    void combine(const NormalEquationsLeastSquares);
  };


}

#endif
