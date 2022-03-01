#ifndef FitAllanAcc_H
#define FitAllanAcc_H

#include <ceres/ceres.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>

namespace imu {

class FitAllanAcc {
  class AllanSigmaError {
   public:
    AllanSigmaError(const double sigma2, const double tau)
        : sigma2_(sigma2), tau_(tau) {}

    template <typename T>
    T calcLog10(T src) const {
      return (log(src)) / (log(10.0));
    }

    template <typename T>
    T calcSigma2(T C_Q, T C_N, T C_B, T C_K, T C_R, T tau) const {
      // clang-format off
            return  C_Q * C_Q / ( tau * tau )
                  + C_N * C_N / tau
                  + C_B * C_B
                  + C_K * C_K * tau
                  + C_R * C_R * tau * tau;
      // clang-format on
    }

    template <typename T>
    bool operator()(const T* const paramt, T* residuals) const {
      T C_Q = T(paramt[0]);
      T C_N = T(paramt[1]);
      T C_B = T(paramt[2]);
      T C_K = T(paramt[3]);
      T C_R = T(paramt[4]);
      T tau = T(tau_);

      T sigma2 = calcSigma2(C_Q, C_N, C_B, C_K, C_R, tau);
      T dsigma2 = T(calcLog10(sigma2)) - T(calcLog10(sigma2_));
      residuals[0] = dsigma2;
      //            std::cout << "_err " << T( sigma2_ ) << " " << sigma2
      //            << std::endl;

      return true;
    }

    double sigma2_;
    double tau_;
  };

 public:
  FitAllanAcc(std::vector<double> sigma2s, std::vector<double> taus,
              double freq);
  std::vector<double> CalculateSimDeviation(
      const std::vector<double> taus) const;
  double getBiasInstability() const;
  double getWhiteNoise() const;

 private:
  std::vector<double> checkData(std::vector<double> sigma2s,
                                std::vector<double> taus);

  std::vector<double> initValue(std::vector<double> sigma2s,
                                std::vector<double> taus);
  double findMinNum(const std::vector<double> num) const;
  int findMinIndex(std::vector<double> num);
  double calcSigma2(double C_Q, double C_N, double C_B, double C_K, double C_R,
                    double tau) const;

 public:
  /**
   * @brief getQ
   *          Quantization Noise
   * @unit: rad
   * @return
   */
  double getQ() const;

  /**
   * @brief getN
   *          Angle Random Walk
   * @unit: rad / sqrt( second )
   * @return
   */
  double getN() const;

  /**
   * @brief getB
   *        Bias Instability
   * @unit: rad / second
   * @return
   */
  double getB() const;

  /**
   * @brief getK
   *      Rate Random Walk
   * @unit: rad / (second*sqrt(second))
   * @return
   */
  double getK() const;

  /**
   * @brief getR
   *        Angle Rate Ramp
   * @unit: rad / (second * second)
   * @return
   */
  double getR() const;

  double C_Q_;
  double C_N_;
  double C_B_;
  double C_K_;
  double C_R_;

 private:
  std::vector<double> m_taus;
  double freq_;
};
}  // namespace imu

#endif  // FitAllanAcc_H
