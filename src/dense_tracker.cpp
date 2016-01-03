#include <iomanip>
#include <assert.h>
#include <sophus/se3.hpp>

#include <dense_tracker.h>
#include <revertable.h>
#include <least_squares.h>

#include <data_types.h>

namespace mySLAM
{
  void computeResiduals(const PointIterator& first_point,
                        const PointIterator& last_point,
                        const RgbdImage& current,
                        const IntrinsicMatrix& intrinsics,
                        const Eigen::Affine3f transform,
                        const Vector8f& reference_weight,
                        const Vector8f& current_weight,
                        ComputeResidualsResult& result);
  void computeResidualsSSE(const PointIterator& first_point,
                           const PointIterator& last_point,
                           const RgbdImage& current,
                           const IntrinsicMatrix& intrinsics,
                           const Eigen::Affine3f transform,
                           const Vector8f& reference_weight,
                           const Vector8f& current_weight,
                           ComputeResidualsResult& result);

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
  Eigen::Matrix2f computeScaleSSE(const ResidualIterator& first_residual,
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
      wcur <<  1.0f / 255.0f,  1.0f, wcur_id * K.fx() / 255.0f, wcur_id * K.fy() / 255.0f,
        wcur_zd * K.fx(), wcur_zd * K.fy(), 0.0f, 0.0f;
      wref << -1.0f / 255.0f, -1.0f, wref_id * K.fx() / 255.0f, wref_id * K.fy() / 255.0f,
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
        computeResidualsSSE(first_point, last_point, cur, K, transformf, wref, wcur, compute_residuals_result);
        // TODO: SSE version. and write the function below?
        size_t n = (compute_residuals_result.last_residual - compute_residuals_result.first_residual);
        std::cout<<"n - "<<n<<std::endl;
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

        precision = computeScaleSSE(compute_residuals_result.first_residual, compute_residuals_result.last_residual, weights.begin(), mean).inverse();
        // std::cout<<precision<<std::endl;
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
      std::cout<< " A = "<< std::endl << A << std::endl;
      std::cout<< " b = "<< std::endl << b << std::endl;

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

  static const __m128 ONES = _mm_set1_ps(1.0f);
  static const __m128 BLEND_MASK = _mm_cmpgt_ps(_mm_setr_ps(0.0f, 1.0f, 0.0f, 0.0f), _mm_set1_ps(0.5f));
  static inline float depthStdDevZ(float depth)
  {
    float sigma_z = depth - 0.4f;
    sigma_z = 0.0012f + 0.0019 * sigma_z * sigma_z;
    // TODO: What this?
    return sigma_z;
  }

  void computeResidualsSSE(const PointIterator& first_point, const PointIterator& last_point, const RgbdImage& current, const IntrinsicMatrix& intrinsics, const Eigen::Affine3f transform, const Vector8f& reference_weight, const Vector8f& current_weight, ComputeResidualsResult& result)
  {
    result.last_point_error = result.first_point_error;
    result.last_residual = result.first_residual;

    Eigen::Matrix<float, 3, 3> K;
    K <<
        intrinsics.fx(), 0, intrinsics.ox(),
        0, intrinsics.fy(), intrinsics.oy(),
        0, 0, 1;

    Eigen::Matrix<float, 3, 4> KT = K * transform.matrix().block<3, 4>(0, 0);

    __m128 kt_r1 = _mm_setr_ps(KT(0, 0), KT(0, 1), KT(0, 2), KT(0, 3));
    __m128 kt_r2 = _mm_setr_ps(KT(1, 0), KT(1, 1), KT(1, 2), KT(1, 3));
    __m128 kt_r3 = _mm_setr_ps(KT(2, 0), KT(2, 1), KT(2, 2), KT(2, 3));

    __m128 current_weight_a = _mm_load_ps(current_weight.data());
    __m128 current_weight_b = _mm_load_ps(current_weight.data() + 4);

    __m128 reference_weight_a = _mm_load_ps(reference_weight.data());
    __m128 reference_weight_b = _mm_load_ps(reference_weight.data() + 4);

    __m128 lower_bound = _mm_set1_ps(0.0f);
    __m128 upper_bound = _mm_setr_ps(current.width - 2, current.height - 2, current.width - 2, current.height - 2);

    EIGEN_ALIGN16 int address[4];

    unsigned int rnd_mode = _MM_GET_ROUNDING_MODE();

    if(rnd_mode != _MM_ROUND_TOWARD_ZERO) _MM_SET_ROUNDING_MODE(_MM_ROUND_TOWARD_ZERO);

    const PointIterator lp = ((last_point - first_point) % 2) != 0 ? last_point - 1 : last_point;

    for(PointIterator p_it = first_point; p_it != lp; p_it += 2)
    {
      // load points
      __m128 p1 = _mm_load_ps((p_it + 0)->point.data);
      __m128 p2 = _mm_load_ps((p_it + 1)->point.data);

      // transform
      __m128 pt1_x = _mm_mul_ps(kt_r1, p1);
      __m128 pt2_x = _mm_mul_ps(kt_r1, p2);

      __m128 pt1_y = _mm_mul_ps(kt_r2, p1);
      __m128 pt2_y = _mm_mul_ps(kt_r2, p2);

      __m128 pt1_z = _mm_mul_ps(kt_r3, p1);
      __m128 pt2_z = _mm_mul_ps(kt_r3, p2);

      __m128 pt1_xy_pt2_xy = _mm_hadd_ps(_mm_hadd_ps(pt1_x, pt1_y), _mm_hadd_ps(pt2_x, pt2_y));
      __m128 pt1_zz_pt2_zz = _mm_hadd_ps(_mm_hadd_ps(pt1_z, pt1_z), _mm_hadd_ps(pt2_z, pt2_z));

      // project
      //__m128 pt1_uv_pt2_uv = _mm_div_ps(pt1_xy_pt2_xy, pt1_zz_pt2_zz);
      __m128 pt1_uv_pt2_uv = _mm_mul_ps(pt1_xy_pt2_xy, _mm_rcp_ps(pt1_zz_pt2_zz));

      // floor
      __m128i pt1_uv_pt2_uv_int = _mm_cvtps_epi32(pt1_uv_pt2_uv);
      __m128 pt1_u0v0_pt2_u0v0 = _mm_cvtepi32_ps(pt1_uv_pt2_uv_int);

      // compute weights
      __m128 pt1w1_uv_pt2w1_uv = _mm_sub_ps(pt1_uv_pt2_uv, pt1_u0v0_pt2_u0v0);
      __m128 pt1w0_uv_pt2w0_uv = _mm_sub_ps(ONES, pt1w1_uv_pt2w1_uv);

      // check image bounds
      int bounds_mask = _mm_movemask_ps(_mm_and_ps(_mm_cmpge_ps(pt1_uv_pt2_uv, lower_bound), _mm_cmple_ps(pt1_uv_pt2_uv, upper_bound)));

      _mm_store_si128((__m128i*) address, pt1_uv_pt2_uv_int);

      if((bounds_mask & 3) == 3)
      {
        const float *x0y0_ptr = current.acceleration.ptr<float>(address[1], address[0]);
        const float *x0y1_ptr = current.acceleration.ptr<float>(address[1] + 1, address[0]);

        _mm_prefetch(x0y1_ptr, _MM_HINT_NTA);

        // shuffle weights
        __m128 w0_uuvv = _mm_unpacklo_ps(pt1w0_uv_pt2w0_uv, pt1w0_uv_pt2w0_uv);
        __m128 w0_uuuu = _mm_unpacklo_ps(w0_uuvv, w0_uuvv);
        __m128 w0_vvvv = _mm_unpackhi_ps(w0_uuvv, w0_uuvv);

        __m128 w1_uuvv = _mm_unpacklo_ps(pt1w1_uv_pt2w1_uv, pt1w1_uv_pt2w1_uv);
        __m128 w1_uuuu = _mm_unpacklo_ps(w1_uuvv, w1_uuvv);
        __m128 w1_vvvv = _mm_unpackhi_ps(w1_uuvv, w1_uuvv);

        // interpolate
        __m128 a1 = _mm_mul_ps(w0_vvvv,
            _mm_add_ps(
                _mm_mul_ps(w0_uuuu, _mm_load_ps(x0y0_ptr + 0)),
                _mm_mul_ps(w1_uuuu, _mm_load_ps(x0y0_ptr + 8))
            )
        );

        __m128 b1 = _mm_mul_ps(w0_vvvv,
            _mm_add_ps(
                _mm_mul_ps(w0_uuuu, _mm_load_ps(x0y0_ptr + 4)),
                _mm_mul_ps(w1_uuuu, _mm_load_ps(x0y0_ptr + 12))
            )
        );

        __m128 a2 = _mm_mul_ps(w1_vvvv,
            _mm_add_ps(
                _mm_mul_ps(w0_uuuu, _mm_load_ps(x0y1_ptr + 0)),
                _mm_mul_ps(w1_uuuu, _mm_load_ps(x0y1_ptr + 8))
            )
        );

        __m128 b2 = _mm_mul_ps(w1_vvvv,
            _mm_add_ps(
                _mm_mul_ps(w0_uuuu, _mm_load_ps(x0y1_ptr + 4)),
                _mm_mul_ps(w1_uuuu, _mm_load_ps(x0y1_ptr + 12))
            )
        );

        // first 4 values in interpolated Vec8f
        __m128 a = _mm_add_ps(a1, a2);
        // last 4 values in interpolated Vec8f
        __m128 b = _mm_add_ps(b1, b2);

        // check for NaNs in interpolated Vec8f
        int nans_mask = _mm_movemask_ps(_mm_cmpunord_ps(a, b));

        if(nans_mask == 0)
        {
          _mm_store_ps(result.last_point_error->point.data, p1);

          __m128 reference_a = _mm_load_ps((p_it + 0)->intensity_and_depth.data);
          // replace the reference image depth value with the transformed one
          reference_a = _mm_or_ps(_mm_and_ps(BLEND_MASK, pt1_zz_pt2_zz), _mm_andnot_ps(BLEND_MASK, reference_a));

          __m128 residual_a = _mm_add_ps(_mm_mul_ps(current_weight_a, a), _mm_mul_ps(reference_weight_a, reference_a));
          _mm_store_ps(result.last_point_error->intensity_and_depth.data + 0, residual_a);

          // occlusion test
          if(result.last_point_error->intensity_and_depth.z > -20.0f * depthStdDevZ((p_it + 0)->intensity_and_depth.z))
          {
            _mm_storel_pi((__m64 *)result.last_residual->data(), residual_a);

            __m128 reference_b = _mm_load_ps((p_it + 0)->intensity_and_depth.data + 4);
            __m128 residual_b = _mm_add_ps(_mm_mul_ps(current_weight_b, b), _mm_mul_ps(reference_weight_b, reference_b));
            _mm_store_ps(result.last_point_error->intensity_and_depth.data + 4, residual_b);

            ++result.last_point_error;
            ++result.last_residual;
          }
          else
          {
            nans_mask = 1;
          }
        }
      }


      if((bounds_mask & 12) == 12)
      {
        const float *x0y0_ptr = current.acceleration.ptr<float>(address[3], address[2]);
        const float *x0y1_ptr = current.acceleration.ptr<float>(address[3] + 1, address[2]);

        _mm_prefetch(x0y1_ptr, _MM_HINT_NTA);

        // shuffle weights
        __m128 w0_uuvv = _mm_unpackhi_ps(pt1w0_uv_pt2w0_uv, pt1w0_uv_pt2w0_uv);
        __m128 w0_uuuu = _mm_unpacklo_ps(w0_uuvv, w0_uuvv);
        __m128 w0_vvvv = _mm_unpackhi_ps(w0_uuvv, w0_uuvv);

        __m128 w1_uuvv = _mm_unpackhi_ps(pt1w1_uv_pt2w1_uv, pt1w1_uv_pt2w1_uv);
        __m128 w1_uuuu = _mm_unpacklo_ps(w1_uuvv, w1_uuvv);
        __m128 w1_vvvv = _mm_unpackhi_ps(w1_uuvv, w1_uuvv);

        // interpolate
        __m128 a1 = _mm_mul_ps(w0_vvvv,
            _mm_add_ps(
                _mm_mul_ps(w0_uuuu, _mm_load_ps(x0y0_ptr + 0)),
                _mm_mul_ps(w1_uuuu, _mm_load_ps(x0y0_ptr + 8))
            )
        );

        __m128 b1 = _mm_mul_ps(w0_vvvv,
            _mm_add_ps(
                _mm_mul_ps(w0_uuuu, _mm_load_ps(x0y0_ptr + 4)),
                _mm_mul_ps(w1_uuuu, _mm_load_ps(x0y0_ptr + 12))
            )
        );

        __m128 a2 = _mm_mul_ps(w1_vvvv,
            _mm_add_ps(
                _mm_mul_ps(w0_uuuu, _mm_load_ps(x0y1_ptr + 0)),
                _mm_mul_ps(w1_uuuu, _mm_load_ps(x0y1_ptr + 8))
            )
        );

        __m128 b2 = _mm_mul_ps(w1_vvvv,
            _mm_add_ps(
                _mm_mul_ps(w0_uuuu, _mm_load_ps(x0y1_ptr + 4)),
                _mm_mul_ps(w1_uuuu, _mm_load_ps(x0y1_ptr + 12))
            )
        );

        // first 4 values in interpolated Vec8f
        __m128 a = _mm_add_ps(a1, a2);
        // last 4 values in interpolated Vec8f
        __m128 b = _mm_add_ps(b1, b2);

        // check for NaNs in interpolated Vec8f
        int nans_mask = _mm_movemask_ps(_mm_cmpunord_ps(a, b));

        if(nans_mask == 0)
        {
          _mm_store_ps(result.last_point_error->point.data, p2);

          __m128 reference_a = _mm_load_ps((p_it + 1)->intensity_and_depth.data);
          // replace the reference image depth value with the transformed one
          reference_a = _mm_or_ps(_mm_and_ps(BLEND_MASK, _mm_unpackhi_ps(pt1_zz_pt2_zz, pt1_zz_pt2_zz)), _mm_andnot_ps(BLEND_MASK, reference_a));

          __m128 residual_a = _mm_add_ps(_mm_mul_ps(current_weight_a, a), _mm_mul_ps(reference_weight_a, reference_a));
          _mm_store_ps(result.last_point_error->intensity_and_depth.data + 0, residual_a);

            // occlusion test
          if(result.last_point_error->intensity_and_depth.z > -20.0f * depthStdDevZ((p_it + 1)->intensity_and_depth.z))
          {
            _mm_storel_pi((__m64 *)result.last_residual->data(), residual_a);

            __m128 reference_b = _mm_load_ps((p_it + 1)->intensity_and_depth.data + 4);
            __m128 residual_b = _mm_add_ps(_mm_mul_ps(current_weight_b, b), _mm_mul_ps(reference_weight_b, reference_b));
            _mm_store_ps(result.last_point_error->intensity_and_depth.data + 4, residual_b);

            ++result.last_point_error;
            ++result.last_residual;
          }
          else
          {
            nans_mask = 1;
          }
        }
      }
    }

    if(rnd_mode != _MM_ROUND_TOWARD_ZERO) _MM_SET_ROUNDING_MODE(rnd_mode);
  }

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
  Eigen::Matrix2f computeScaleSSE(const ResidualIterator& first_residual, const ResidualIterator& last_residual, const WeightIterator& first_weight, const Eigen::Vector2f& mean)
  {
    const ResidualIterator lr = last_residual - ((last_residual - first_residual) % 2);

    WeightIterator w_it = first_weight;
    size_t n = (last_residual - first_residual);
    float scale = 1.0f / (n - 2 -1);

    __m128 cov_acc = _mm_setzero_ps();
    __m128 s = _mm_set1_ps(scale);
    __m128 mean2 = _mm_setr_ps(mean(0), mean(1), mean(0), mean(1));

    __m128 fac1, fac2, w;
    for(ResidualIterator err_it = first_residual;err_it != lr; err_it += 2, w_it += 2)
    {
      __m128 r1r2 = _mm_load_ps(err_it->data());
      __m128 diff_r1r2 = _mm_sub_ps(r1r2, mean2); // [x1, y1, x2, y2]

      fac1 = _mm_movelh_ps(diff_r1r2, diff_r1r2); // [x1, y1, x1, y1]
      fac2 = _mm_unpacklo_ps(diff_r1r2, diff_r1r2); // [x1, x1, y1, y1]
      w = _mm_set1_ps(*(w_it + 0));

      __m128 p1 = _mm_mul_ps(s, _mm_mul_ps(w, _mm_mul_ps(fac1, fac2)));

      fac1 = _mm_movelh_ps(diff_r1r2, diff_r1r2); // [x2, y2, x2, y2]
      fac2 = _mm_unpacklo_ps(diff_r1r2, diff_r1r2); // [x2, x2, y2, y2]
      w = _mm_set1_ps(*(w_it + 1));

      __m128 p2 = _mm_mul_ps(s, _mm_mul_ps(w, _mm_mul_ps(fac1, fac2)));

      cov_acc = _mm_add_ps(cov_acc, _mm_add_ps(p1, p2));
    }

    EIGEN_ALIGN16 float tmp[4];
    _mm_store_ps(tmp, cov_acc);

    Eigen::Matrix2f covariance;
    covariance(0, 0) = tmp[0];
    covariance(0, 1) = tmp[1];
    covariance(1, 0) = tmp[1];
    covariance(1, 1) = tmp[3];

    for(ResidualIterator err_it = lr;err_it != last_residual; ++err_it, ++w_it)
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
