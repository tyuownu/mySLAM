#include "least_squares.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <math_sse.h>

namespace mySLAM
{
  // begin LeastSquaresInterface function
  LeastSquaresInterface::~LeastSquaresInterface()
  {}
  // end LeastSquaresInterface function
  // begin NormalEquationsLeastSquares function
  NormalEquationsLeastSquares::~NormalEquationsLeastSquares()
  {}

  void NormalEquationsLeastSquares::initialize(const size_t maxnum_constraints)
  {
    A.setZero();
    A_opt.setZero();
    b.setZero();
    error = 0;
    this->num_constraints = 0;
    this->maxnum_constraints = maxnum_constraints;
  }

  void NormalEquationsLeastSquares::update(const mySLAM::Vector6& J, const NumType& res, const NumType& weight)
  {
    NumType factor = weight;
    A_opt.rankUpdate(J, factor);
    MathSse<Sse::Enabled, NumType>::add(b, J, -res * factor);
    num_constraints += 1;
  }

  void NormalEquationsLeastSquares::update(const Eigen::Matrix<NumType, 2, 6>& J,
                                           const Eigen::Matrix<NumType, 2, 1>& res,
                                           const Eigen::Matrix<NumType, 2, 2>& weight)
  {
    A_opt.rankUpdate(J, weight);
    b -= J.transpose() * weight * res;
    num_constraints += 1;
  }

  void NormalEquationsLeastSquares::finish()
  {
    A_opt.toEigen(A);
  }

  void mySLAM::NormalEquationsLeastSquares::solve(mySLAM::Vector6& x)
  {
    x = A.ldlt().solve(b);
  }
  // end NormalEquationsLeastSquares function
} // end namespace mySLAM
