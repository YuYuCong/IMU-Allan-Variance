#ifndef ALLAN_H
#define ALLAN_H

#include <math.h>
#include <iostream>
#include <vector>
#include "../../include/type.h"

namespace imu {

class AllanGyr {
 public:
  AllanGyr(std::string name, int maxCluster = 10000);
  ~AllanGyr();

  /**
   * @brief push gyro reading in rad into sequence
   * @param data gyro reading in rad per second
   * @param time
   */
  void PushRadPerSec(double data, double time);

  /**
   * @brief push gyro reading in degree into sequence
   * @param data gyro reading in degree per second
   * @param time
   */
  void PushDegreePerSec(double data, double time);

  /**
   * @brief push gyro reading in degree into sequence
   * @param data gyro reading in degree per hour
   * @param time
   */
  void PushDegreePerHou(double data, double time);

  void CalculateAllanVariance();

  std::vector<double> GetVariance() const;
  std::vector<double> GetDeviation();
  std::vector<double> GetTimestamp();
  std::vector<int> getFactors() const;
  double GetAvgValue();
  double GetFrequency() const;

 private:
  std::vector<double> calcVariance(double period);

  std::vector<double> calcThetas(const double freq);
  void initStrides();
  std::vector<double> getLogSpace(float a, float b);
  double getAvgFreq() { return 1.0 / getAvgDt(); }
  double getAvgDt();
  double getAvgPeriod() { return getAvgDt(); }
  int getFactorsNum() { return numFactors; }

  std::string m_name;
  double m_freq;
  int numData;
  std::vector<GyrData> m_rawData;
  std::vector<double> m_thetas;
  int numCluster;
  int numFactors;
  std::vector<int> mFactors;

  std::vector<double> mVariance;
};
}  // namespace imu

#endif  // ALLAN_H
