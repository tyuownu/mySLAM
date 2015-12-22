#ifndef _DENSE_TRACKER_H
#define _DENSE_TRACKER_H

#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <data_types.h>
#include <intrinsic_matrix.h>
#include <rgbd_image.h>
#include <point_selection.h>
#include <math_sse.h>
// TODO to be adding

namespace mySLAM
{
  class DenseTracker
  {
  public:
    struct Config
    {
      int FirstLevel, LastLevel;
      int MaxIterationsPerLevel;
      double Precision;
      double Mu;

      bool UseInitialEstimate;
      bool UseWeighting;

//      bool UseParallel;

      // TODO, two distribution.
      float IntensityDerivativeThreshold;
      float DepthDerivativeThreshold;

      Config();
      size_t getNumLevels() const;
      bool UseEstimateSmoothing() const;
      bool IsSane() const;
    };

    struct TerminationCriteria
    {
      enum Enum
      {
        IterationsExceeded,
        IncrementTooSmall,
        LogLikelihoodDecreased,
        TooFewConstraints,
        NumCriteria
      };
    };

    struct IterationStats
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      size_t Id, ValidConstraints;
      double TDistributionLogLikelihood;
      Eigen::Vector2d TDistributionMean;
      Eigen::Matrix2d TDistributionPrecision;

      double PriorLogLikelihood;
      mySLAM::Vector6d EstimateIncrement;
      mySLAM::Matrix6d EstimateInformation;

      void InformationEigenValues(mySLAM::Vector6d& eigenvalues) const;
      double InformationConditionNumber() const;
    };

    typedef std::vector<IterationStats, Eigen::aligned_allocator<IterationStats> > IterationStatsVector;

    struct LevelStats
    {
      size_t Id, MaxValidPixels, ValidPixels;
      TerminationCriteria::Enum TerminationCriterion;
      IterationStatsVector Iterations;

      bool HasIterationWithIncrement() const;
      IterationStats& LastIterationWithIncrement();
      IterationStats& LastTteration();

      const IterationStats& LastIterationWithIncrement() const;
      const IterationStats& LastIteration() const;
    };
    typedef std::vector<LevelStats> LevelStatsVector;

    struct Stats
    {
      LevelStatsVector Levels;
    };

    struct Result
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      mySLAM::AffineTransformd Transformation;
      mySLAM::Matrix6d         Information;
      double LogLikelihood;
      Stats Statistics;

      Result();

      bool isNaN() const;
      void setIdentity();
      void clearStatistics();
    };

    static const Config& getDefaultConfig();
    Config& setConfig();

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    DenseTracker(const Config& cfg = getDefaultConfig());
    DenseTracker(const mySLAM::DenseTracker& other);

    const Config& configuration() const
    {
      return cfg;
    }
    void configure(const Config& cfg);
    bool match(mySLAM::RgbdImagePyramid& reference,
               mySLAM::RgbdImagePyramid& current,
               mySLAM::AffineTransformd& transformation);
    // bool match(mySLAM::PointSelection& reference,
    //            mySLAM::RgbdImagePyramid& current,
    //            mySLAM::AffineTransformd& Transformation);
    // bool match(mySLAM::RgbdImagePyramid& reference,
    //            mySLAM::RgbdImagePyramid& current,
    //            mySLAM::DenseTracker::Result& result);
    bool match(mySLAM::PointSelection& reference,
               mySLAM::RgbdImagePyramid& current,
               mySLAM::DenseTracker::Result& result);

    cv::Mat computeIntensityErrorImage(mySLAM::RgbdImagePyramid& reference,
                                       mySLAM::RgbdImagePyramid& current,
                                       const mySLAM::AffineTransform& transformation,
                                       size_t level = 0);

    static inline void computeJacobianOfProjectionAndTransformation(const mySLAM::Vector4& p, mySLAM::Matrix2x6& jacobian);
    static inline void compute3rdRowOfJacobianOfTransformation(const mySLAM::Vector4& p, mySLAM::Vector6& j);
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > ResidualVectorType;
    typedef std::vector<float> WeightVectorType;

    private:
    struct IterationContext
    {
      const Config& cfg;

      int Level;
      int Iteration;
      double Error, LastError;
      IterationContext(const Config& cfg);

      bool IsFirstIteration() const;
      bool IsFirstIterationOnLevel() const;
      bool IsFirstLevel() const;
      bool IsLastLevel() const;
      bool IterationsExceeded() const;

      double ErrorDiff() const;
    };

    Config cfg;
    IterationContext itctx_;
    // mySLAM::WeightCalculation weight_calculation_;
    // mySLAM::PointSelection reference_selection_;
    // mySLAM::ValidPointAndGradientThresholdPredicate selection_predicate_;
    mySLAM::PointWithIntensityAndDepth::VectorType points, points_error;

    ResidualVectorType residuals;
    WeightVectorType weights;

  };

  typedef PointWithIntensityAndDepth::VectorType::iterator PointIterator;
  typedef DenseTracker::ResidualVectorType::iterator ResidualIterator;
  typedef DenseTracker::WeightVectorType::iterator WeightIterator;
  typedef std::vector<uint8_t>::iterator ValidFlagIterator;

  struct ComputeResidualsResult
  {
    PointIterator first_point_error, last_point_error;
    ResidualIterator first_residual, last_residual;
    ValidFlagIterator first_valid_flag, last_valid_flag;
  };

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
} // end namespace mySLAM

#endif
