#include <iomanip>
#include <assert.h>
#include <sophus/se3.hpp>

#include <dense_tracker.h>
#include <revertable.h>
#include <least_squares.h>

#include <data_types.h>

namespace mySLAM
{
  static inline float computeWeight(const Eigen::Vector2f& r, const Eigen::Vector2f& mean, const Eigen::Matrix2f& precision);
  void computeWeights(const ResidualIterator& first_residual,
                      const ResidualIterator& last_residual,
                      const WeightIterator& first_weight,
                      const Eigen::Vector2f& mean,
                      const Eigen::Matrix2f& precision);
  static inline Eigen::Matrix2f computeScale(const float& weight, const Eigen::Vector2f& r, const Eigen::Vector2f* mean);
  Eigen::Matrix2f computeScales(const ResidualIterator& first_residual,
                                const ResidualIterator& last_residual,
                                const WeightIterator& first_weight,
                                const Eigen::Vector2f& mean);
  float computeCompleteDataLogLikelihood(const ResidualIterator& first_residual,
                                         const ResidualIterator& last_residual,
                                         const WeightIterator& first_weight,
                                         const Eigen::Vector2f& mean,
                                         const Eigen::Matrix2f& precision);
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

  // begin DenseTracker::Result function
  DenseTracker::Result::Result() :
    LogLikelihood(std::numeric_limits<double>::max())
  {
    double nan = std::numeric_limits<double>::quiet_NaN();
    Transformation.linear().setConstant(nan);
    Transformation.translation().setConstant(nan);
    Information.setIdentity();
  }
  bool DenseTracker::Result::isNaN() const
  {
    return !std::isfinite(Transformation.matrix().sum()) ||
      !std::isfinite(Information.sum());
  }

  void DenseTracker::Result::setIdentity()
  {
    Transformation.setIdentity();
    Information.setIdentity();
    LogLikelihood = 0.0f;   // TODO: what is this?
  }
  // end DenseTracker::Result function

  // begin DenseTracker::IterationContext function
  DenseTracker::IterationContext::IterationContext(const DenseTracker::Config& cfg) :
    cfg(cfg)
  {}
  bool DenseTracker::IterationContext::IsFirstIterationOnLevel() const
  {
    return 0 == Iteration;
  }

  bool DenseTracker::IterationContext::IterationsExceeded() const
  {
    int max_iterations = cfg.MaxIterationsPerLevel;
    return Iteration >= max_iterations;
  }
  // end DenseTracker::IterationContext function
  // begin DenseTracker function
  DenseTracker::DenseTracker(const Config& config) :
    itctx_(cfg)
  {}
  const DenseTracker::Config& DenseTracker::getDefaultConfig()
  {
    static Config defaultConfig;
    return defaultConfig;
  }

  void DenseTracker::configure(const Config& config)
  {
    assert(config.IsSane());
  }
  // match 1
  // match 2
  // match 3
  // match 4
  bool DenseTracker::match(mySLAM::PointSelection& reference,
                           mySLAM::RgbdImagePyramid& current,
                           mySLAM::DenseTracker::Result& result)
  {
    current.compute(cfg.getNumLevels());
    bool success = true;
    if(cfg.UseInitialEstimate)  // maybe can be delete in the future
    {
      assert(!result.isNaN() && "Initialization is NaN");
    }
    else
    {
      result.setIdentity();
    }

    Sophus::SE3d inc(result.Transformation.rotation(), result.Transformation.translation());

    Revertable<Sophus::SE3d> initial(inc);
    Revertable<Sophus::SE3d> estimate;
    bool accept = true;

    if(points_error.size() < reference.getMaximumNumberOfPoint(cfg.LastLevel))
      points_error.resize(reference.getMaximumNumberOfPoint(cfg.LastLevel));
    if(residuals.size() < reference.getMaximumNumberOfPoint(cfg.LastLevel))
      residuals.resize(reference.getMaximumNumberOfPoint(cfg.LastLevel));
    if(weights.size() < reference.getMaximumNumberOfPoint(cfg.LastLevel))
      weights.resize(reference.getMaximumNumberOfPoint(cfg.LastLevel));

    std::vector<uint8_t> valid_residuals;

    Eigen::Vector2f mean;
    mean.setZero();
    Eigen::Matrix2f precision;
    precision.setZero();

    // In this experiment, FistLevel = 3, and LastLevel = 1. That is, from 80*60 -> 320*240.
    for(itctx_.Level = cfg.FirstLevel; itctx_.Level >= cfg.LastLevel; --itctx_.Level)
    {
      result.Statistics.Levels.push_back(LevelStats());
      LevelStats& level_stats = result.Statistics.Levels.back();

      mean.setZero(); precision.setZero();

      itctx_.Iteration = 0;
      itctx_.Error = std::numeric_limits<double>::max();

      RgbdImage& cur = current.level(itctx_.Level);  // get current image.
      const IntrinsicMatrix& K = cur.camera().intrinsics();

      Vector8f wcur, wref;
      float wcur_id = 0.5f, wref_id = 0.5f, wcur_zd = 1.0f, wref_zd = 0.0f;
      wcur << 1.0f / 255.0f, 1.0f, wcur_id * K.fx() / 255.0f, wcur_id * K.fy() / 255.0f,
        wcur_zd * K.fx(), wcur_zd * K.fy(), 0.0f, 0.0f;
      wref << 1.0f / 255.0f, 1.0f, wref_id * K.fx() / 255.0f, wref_id * K.fy() / 255.0f,
        wref_zd * K.fx(), wref_zd * K.fy(), 0.0f, 0.0f;

      PointSelection::PointIterator first_point, last_point;
      reference.select(itctx_.Level, first_point, last_point);
      cur.buildAccelerationStructure();

      level_stats.Id = itctx_.Level;
      level_stats.MaxValidPixels = reference.getMaximumNumberOfPoint(itctx_.Level);
      level_stats.ValidPixels = last_point - first_point;

      NormalEquationsLeastSquares ls;
      Matrix6d A;
      Vector6d x, b;
      x = inc.log();

      ComputeResidualsResult compute_residuals_result;
      compute_residuals_result.first_point_error = points_error.begin();
      compute_residuals_result.first_residual    = residuals.begin();
      compute_residuals_result.first_valid_flag  = valid_residuals.begin();

      do
      {
        level_stats.Iterations.push_back(IterationStats());
        IterationStats& iteration_stats = level_stats.Iterations.back();
        iteration_stats.Id = itctx_.Iteration;

        double total_error = 0.0f;
        Eigen::Affine3f transformf;

        inc = Sophus::SE3d::exp(x);
        initial.update() = inc.inverse() * initial();
        estimate.update() = inc * estimate();
        transformf = estimate().matrix().cast<float>();
        computeResiduals(first_point, last_point, cur, K, transformf, wref, wcur, compute_residuals_result);
        // TODO: SSE version. and write the function below?
        size_t n = (compute_residuals_result.last_residual - compute_residuals_result.first_residual);
        iteration_stats.ValidConstraints = n;

        if(n < 6)
        {
          initial.revert();
          estimate.revert();
          level_stats.TerminationCriterion = TerminationCriteria::TooFewConstraints;
          break;
        }

        if(itctx_.IsFirstIterationOnLevel())
        {
          std::fill(weights.begin(), weights.begin() + n, 1.0f);
        }
        else
        {
          computeWeights(compute_residuals_result.first_residual, compute_residuals_result.last_residual, weights.begin(), mean, precision);
        }

        precision = computeScales(compute_residuals_result.first_residual, compute_residuals_result.last_residual, weights.begin(), mean).inverse();
        float ll = computeCompleteDataLogLikelihood(compute_residuals_result.first_residual, compute_residuals_result.last_residual, weights.begin(), mean, precision);

        iteration_stats.TDistributionLogLikelihood = -ll;
        iteration_stats.TDistributionMean          = mean.cast<double>();
        iteration_stats.TDistributionPrecision     = precision.cast<double>();
        iteration_stats.PriorLogLikelihood         = cfg.Mu * initial().log().squaredNorm();

        total_error      = -ll;
        itctx_.LastError = itctx_.Error;
        itctx_.Error     = total_error;

        accept = itctx_.Error < itctx_.LastError;
        if (!accept)
        {
          initial.revert();
          estimate.revert();
          level_stats.TerminationCriterion = TerminationCriteria::LogLikelihoodDecreased;
          break;
        }

        WeightVectorType::iterator w_it = weights.begin();
        Matrix2x6 J, Jw;
        Eigen::Vector2f Ji;
        Vector6 Jz;
        ls.initialize(1);
        for(PointIterator e_it = compute_residuals_result.first_point_error; e_it != compute_residuals_result.last_point_error; ++e_it, ++w_it)
        {
          computeJacobianOfProjectionAndTransformation(e_it->getPointVec4f(), Jw);
          compute3rdRowOfJacobianOfTransformation(e_it->getPointVec4f(), Jz);
          J.row(0) = e_it->getIntensityDerivativeVec2f().transpose() * Jw;
          J.row(1) = e_it->getDepthDerivativeVec2f().transpose() * Jw - Jz.transpose();
          ls.update(J, e_it->getIntensityAndDepthVec2f(), (*w_it) * precision);
        }
        ls.finish();

        A = ls.A.cast<double>() + cfg.Mu * Matrix6d::Identity();
        b = ls.b.cast<double>() + cfg.Mu * initial().log();
        x = A.ldlt().solve(b);

        iteration_stats.EstimateIncrement = x;
        iteration_stats.EstimateInformation = A;

        itctx_.Iteration++;
      }
      while(accept && x.lpNorm<Eigen::Infinity>() > cfg.Precision && !itctx_.IterationsExceeded());

      if (x.lpNorm<Eigen::Infinity>() <= cfg.Precision)
      {
        level_stats.TerminationCriterion = TerminationCriteria::IncrementTooSmall;
      }
      if (itctx_.IterationsExceeded())
      {
        level_stats.TerminationCriterion = TerminationCriteria::IterationsExceeded;
      }
    }
    LevelStats& last_level = result.Statistics.Levels.back();
    IterationStats& last_iteration = last_level.TerminationCriterion != TerminationCriteria::LogLikelihoodDecreased
      ? last_level.Iterations[last_level.Iterations.size() - 1] : last_level.Iterations[last_level.Iterations.size() - 2];

    result.Transformation = estimate().inverse().matrix();
    result.Information    = last_iteration.EstimateInformation * 0.008 * 0.008;
    result.LogLikelihood  = last_iteration.TDistributionLogLikelihood + last_iteration.PriorLogLikelihood;

    return success;

  }

  inline void DenseTracker::computeJacobianOfProjectionAndTransformation(const Vector4& p, Matrix2x6& j)
  {
    NumType z = 1.0f / p(2);
    NumType z_sqr = 1.0f / (p(2) * p(2));

    j(0, 0) = z;
    j(0, 1) = 0.0f;
    j(0, 2) = -p(0) * z_sqr;
    j(0, 3) = j(0, 2) * p(1);
    j(0, 4) = 1.0f - j(0, 2) * p(0);
    j(0, 5) = -p(1) * z;

    j(1, 0) = 0.0f;
    j(1, 1) = z;
    j(1, 2) = -p(1) * z_sqr;
    j(1, 3) = -1.0f + j(1, 2) * p(1);
    j(1, 4) = -j(0, 3);
    j(1, 5) = p(0) * z;
  }

  inline void DenseTracker::compute3rdRowOfJacobianOfTransformation(const Vector4& p, Vector6& j)
  {
    j(0) = 0.0f;
    j(1) = 0.0f;
    j(2) = 1.0f;
    j(3) = p(1);
    j(4) = -p(0);
    j(5) = 0.0f;
  }
  // end DenseTracker function


  void computeResiduals(const PointIterator& first_point,
                        const PointIterator& last_point,
                        const RgbdImage& current,
                        const IntrinsicMatrix& intrinsics,
                        const Eigen::Affine3f transform,
                        const Vector8f& reference_weight,
                        const Vector8f& current_weight,
                        ComputeResidualsResult& result)
  {
    result.last_point_error = result.first_point_error;
    result.last_residual    = result.first_residual;

    Eigen::Matrix<float, 3, 3> K;
    K <<
      intrinsics.fx(), 0, intrinsics.ox(),
      0, intrinsics.fy(), intrinsics.oy(),
      0, 0, 1;
    Eigen::Matrix<float, 3, 4> KT = K * transform.matrix().block<3, 4>(0, 0);
    Eigen::Vector4f transformed_point;
    transformed_point.setConstant(1);

    float t = std::numeric_limits<float>::quiet_NaN();
    for(PointIterator p_it = first_point; p_it != last_point; p_it++)
    {
      transformed_point.head<3>() = KT * p_it->getPointVec4f();
      float projected_x = transformed_point(0) / transformed_point(2);
      float projected_y = transformed_point(1) / transformed_point(2);

      if(!current.inImage(projected_x, projected_y) || !current.inImage(projected_x + 1, projected_y + 1)) continue;
      float x0 = std::floor(projected_x);
      float y0 = std::floor(projected_y);

      float x0w, x1w, y0w, y1w;
      x1w = projected_x - x0;
      x0w = 1.0f - x1w;
      y1w = projected_y - y0;
      y0w = 1.0f - y1w;

      const float *x0y0_ptr = current.acceleration.ptr<float>(int(y0), int(x0));
      const float *x0y1_ptr = current.acceleration.ptr<float>(int(y0 + 1), int(x0));
      Vector8f::ConstAlignedMapType x0y0(x0y0_ptr);
      Vector8f::ConstAlignedMapType x1y0(x0y0_ptr + 8);
      Vector8f::ConstAlignedMapType x0y1(x0y1_ptr);
      Vector8f::ConstAlignedMapType x1y1(x0y1_ptr + 8);
      Vector8f interpolated = (x0y0 * x0w + x1y0 * x1w) * y0w + (x0y1 * x0w + x1y1 * x1w) * y1w;
      if(!std::isfinite(interpolated(1)) || !std::isfinite(interpolated(4)) || !std::isfinite(interpolated(5)))
        continue;

      result.last_point_error->getPointVec4f() = p_it->getPointVec4f();
      Vector8f reference = p_it->getIntensityAndDepthWithDerivativesVec8f();
      reference(1) = transformed_point(2);

      result.last_point_error->getIntensityAndDepthWithDerivativesVec8f() =
        current_weight.cwiseProduct(interpolated) +
        reference_weight.cwiseProduct(reference);
      *result.last_residual = result.last_point_error->getIntensityAndDepthVec2f();

      ++result.last_point_error;
      ++result.last_residual;
    }
  }

  // end DenseTracker function

  static inline float computeWeight(const Eigen::Vector2f& r, const Eigen::Vector2f& mean, const Eigen::Matrix2f& precision)
  {
    Eigen::Vector2f diff = r - mean;
    return (2.0f + 5.0f) / (5.0f + diff.transpose() * precision * diff); // TODO: To know the means.
  }

  void computeWeights(const ResidualIterator& first_residual,
                      const ResidualIterator& last_residual,
                      const WeightIterator& first_weight,
                      const Eigen::Vector2f& mean,
                      const Eigen::Matrix2f& precision)
  {
    Eigen::Vector2f diff;
    WeightIterator w_it = first_weight;
    for(ResidualIterator err_it = first_residual; err_it != last_residual; ++err_it, ++w_it)
    {
      *w_it = computeWeight(*err_it, mean, precision);
    }
  }

  static inline Eigen::Matrix2f computeScale(const float& weight, const Eigen::Vector2f& r, const Eigen::Vector2f& mean)
  {
    Eigen::Vector2f diff;
    diff = r - mean;
    return weight * diff * diff.transpose();
  }

  Eigen::Matrix2f computeScales(const ResidualIterator& first_residual,
                                const ResidualIterator& last_residual,
                                const WeightIterator& first_weight,
                                const Eigen::Vector2f& mean)
  {
    Eigen::Matrix2f covariance;
    covariance.setZero();
    WeightIterator w_it = first_weight;
    size_t n = (last_residual - first_residual);
    float scale = 1.0f / (n - 2 - 1);
    for(ResidualIterator err_it = first_residual; err_it != last_residual; ++err_it, ++w_it)
    {
      covariance += scale * computeScale(*w_it, *err_it, mean);
    }
    return covariance;
  }

  float computeCompleteDataLogLikelihood(const ResidualIterator& first_residual,
                                         const ResidualIterator& last_residual,
                                         const WeightIterator& first_weight,
                                         const Eigen::Vector2f& mean,
                                         const Eigen::Matrix2f& precision)
  {
    size_t n = (last_residual - first_residual);
    size_t c = 1;
    double error_sum = 0.0f;
    double error_acc = 1.0f;
    for(ResidualIterator err_it = first_residual; err_it != last_residual; ++err_it, ++c)
    {
      error_acc *= (1.0f + 0.2f * ((*err_it).transpose() * precision * (*err_it))(0, 0));
      if(0 == (c % 50))
      {
        error_sum += std::log(error_acc);
        error_acc = 1.0f;
      }
    }
    return 0.5f * n * std::log(precision.determinant()) - 0.5f * (5.0f + 2.0f) * error_sum;
  }

} // end namespace mySLAM
