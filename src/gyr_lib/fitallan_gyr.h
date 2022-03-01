#ifndef FitAllanGyr_H
#define FitAllanGyr_H

#include <ceres/ceres.h>
#include <cmath>
#include <eigen3/Eigen/Eigen>

namespace imu {

class FitAllanGyr {
  class AllanSigmaError {
   public:
    AllanSigmaError(const double sigma2, const double tau)
        : sigma2_(sigma2), tau_(tau) {}

    template <typename T>
    T calcLog10(T src) const {
      return (log(src)) / (log(10.0));
    }

    /**
     * Calculates the total Allan variance (\sigma^2_A(\tau)) according to
     * Equation (8) in [1]:
     *
     * \sigma^2_A(\tau) = \sum_{n=-2}^{2}{C_n \tau^n}
     *
     * [1]: D. Li , J. Wang , S. Babu , Z. L. Xiong: "Nonlinear Stochastic
     * Modeling for INS Derived Doppler Estimates" (2005).
     * http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.73.5605
     */
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
      // std::cout << "_err " << T( sigma2_ ) << " " << sigma2 << std::endl;

      return true;
    }

    double sigma2_;
    double tau_;
  };

 public:
  FitAllanGyr(std::vector<double> sigma2s, std::vector<double> taus,
              double freq);
  std::vector<double> CalculateSimDeviation(
      const std::vector<double> taus) const;

  /**
   * The following functions return the discrete-time standard deviations at the
   * given frequency.
   */
  double getBiasInstability() const;
  double getWhiteNoise() const;
  double getRandomWalk() const;

 private:
  std::vector<double> initValue(std::vector<double> sigma2s,
                                std::vector<double> taus);
  double findMinNum(const std::vector<double> num) const;
  int findMinIndex(std::vector<double> num);

  /**
   * Calculates the total Allan variance (\sigma^2_A(\tau)) according to
   * Equation (8) in [1]:
   *
   * \sigma^2_A(\tau) = \sum_{n=-2}^{2}{C_n \tau^n}
   *
   * [1]: D. Li , J. Wang , S. Babu , Z. L. Xiong: "Nonlinear Stochastic
   * Modeling for INS Derived Doppler Estimates" (2005).
   * http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.73.5605
   */
  double calcSigma2(double C_Q, double C_N, double C_B, double C_K, double C_R,
                    double tau) const;

 public:
  /**
   * @brief getQ
   *          Quantization Noise, a.k.a. Angle Quantization
   * @unit: rad
   *
   * The methods getQ(), getN(), getB(), getK() and getR() convert the factors
   * C_{-2}, C_{-1}, ..., C_{2} into the coefficients of the IMU error models Q,
   * N, B, K, R according to Equation (9) in [1]:
   *
   * [1]: D. Li , J. Wang , S. Babu , Z. L. Xiong: "Nonlinear Stochastic
   * Modeling for INS Derived Doppler Estimates" (2005).
   * http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.73.5605
   *
   * The coefficients Q, N, B, K, R are continuous-time, i.e., independent of
   * the sampling rate. To convert them into discrete-time standard deviations,
   * see getBiasInstability(), getWhiteNoise() etc.
   *
   * @return
   */
  double getQ() const;

  // clang-format off
  /**
   * @brief getN
   *          White Noise / White Rate Noise, a.k.a. Angle Random Walk, Noise Density / Rate Noise Density
   *          Kalibr: \sigma_g, Gyroscope "white noise", gyroscope_noise_density (see https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
   *          equivalent units: rad / s / sqrt(Hz)
   * @unit: rad / sqrt( second )
   * @return
   */
  // clang-format on
  double getN() const;

  /**
   * @brief getB
   *        Bias Instability, a.k.a. In-Run Bias Stability, Flicker Noise
   * @unit: rad / second
   * @return
   */
  double getB() const;

  /**
   * @brief getK
   *      Random Walk, a.k.a. Rate Random Walk, Bias
   *      Kalibr: \sigma_{bg}, Gyroscope "random walk", gyroscope_random_walk
   *      equivalent units: rad / s^2 / sqrt(Hz)
   * @unit: rad / (second*sqrt(second))
   * @return
   */

  double getK() const;
  /**
   * @brief getR
   *        Angle Rate Ramp, a.k.a. Ramp Instability
   * @unit: rad / s^2
   * @return
   */
  double getR() const;

  /**
   * The following variables are the factors C_{-2}, C_{-1}, ..., C_{2} from
   * Equation (8) in [1]:
   *
   * \sigma^2_A(\tau) = \sum_{n=-2}^{2}{C_n \tau^n}
   *
   * [1]: D. Li , J. Wang , S. Babu , Z. L. Xiong: "Nonlinear Stochastic
   * Modeling for INS Derived Doppler Estimates" (2005).
   * http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.73.5605
   */
  double C_Q_;  // = C_{-2}
  double C_N_;  // = C_{-1}
  double C_B_;  // = C_{0}
  double C_K_;  // = C_{1}
  double C_R_;  // = C_{2}

 private:
  std::vector<double> m_taus;
  double freq_;
};
}  // namespace imu
#endif  // FitAllanGyr_H
