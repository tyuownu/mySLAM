#ifndef _MATH_SSE_H
#define _MATH_SSE_H

#include <Eigen/Core>

namespace mySLAM
{
  /**
   * As the author said, a 6x6 self adjoint matrix with optimized "rankUpdate(u, scale)" (10x faster than Eigen impl, 1.8x faster than MathSse::addOuterProduct(...))
   */
  class OptimizedSelfAdjointMatrix6x6f
  {
  public:
    OptimizedSelfAdjointMatrix6x6f();
    void rankUpdate(const Eigen::Matrix<float, 6, 1>& u, const float& alpha);
    void rankUpdate(const Eigen::Matrix<float, 2, 6>& u, const Eigen::Matrix2f& alpha);
    void operator +=(const OptimizedSelfAdjointMatrix6x6f& other);
    void setZero();
    void toEigen(Eigen::Matrix<float, 6, 6>& m) const;

  private:
    enum
    {
      Size = 24
    };
    EIGEN_ALIGN16 float data[Size];
  };

  struct Sse
  {
    enum
    {
      Enabled,
      Disabled
    };
  };

  template<int Enabled, typename T>
  class MathSse
  {
  public:
    static void addOuterProduct(Eigen::Matrix<T, 6, 6>& mat, const Eigen::Matrix<T, 6, 1>& vec, const T& scale);
    static void add(Eigen::Matrix<T, 6, 1>& vec, const Eigen::Matrix<T, 6, 1>& other, const T& scale);

    // static void addOuterProduct(Eigen::Matrix<float, 6, 6>& mat, const Eigen::Matrix<float, 6, 1>& vec, const float& scale);
    // static void addOuterProduct(Eigen::Matrix<double, 6, 6>& mat, const Eigen::Matrix<double, 6, 1>& vec, const double& scale);
    // static void add(Eigen::Matrix<float, 6, 1>& vec, const Eigen::Matrix<float, 6, 1>& other, const float& scale);
    // static void add(Eigen::Matrix<double, 6, 1>& vec, const Eigen::Matrix<double, 6, 1>& other, const double& scale);

  private:
    MathSse() {}
    MathSse(const MathSse& other) {}
    ~MathSse() {}
  };

  // template<typename T>
  // class MathSse<Sse::Disabled, T>
  // {
  // public:
  //   static void addOuterproduct(Eigen::Matrix<T, 6, 6>& mat, const Eigen::Matrix<T, 6, 1>& vec, const T& scale)
  //   {
  //     mat += vec * vec.transpose() * scale;
  //   }
  //   static void add(Eigen::Matrix<T, 6, 1>& vec, const Eigen::Matrix<T, 6, 1>& other, const T& scale)
  //   {
  //     vec += other * scale;
  //   }

  // private:
  //   MathSse() {}
  //   MathSse(const MathSse& other) {}
  //   ~MathSse() {}
  // };

  template<>
  void MathSse<Sse::Enabled, float>::addOuterProduct(Eigen::Matrix<float, 6, 6>& mat, const Eigen::Matrix<float, 6, 1>& vec, const float& scale);

  template<>
  void MathSse<Sse::Enabled, double>::addOuterProduct(Eigen::Matrix<double, 6, 6>& mat, const Eigen::Matrix<double, 6, 1>& vec, const double& scale);

  template<>
  void MathSse<Sse::Enabled, float>::add(Eigen::Matrix<float, 6, 1>& vec, const Eigen::Matrix<float, 6, 1>& other, const float& scale);

  template<>
  void MathSse<Sse::Enabled, double>::add(Eigen::Matrix<double, 6, 1>& vec, const Eigen::Matrix<double, 6, 1>& other, const double& scale);

} // end namespace mySLAM


#endif
