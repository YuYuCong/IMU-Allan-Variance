#ifndef AllanAcc_H
#define AllanAcc_H

#include <math.h>
#include <iostream>
#include <vector>
#include "../../include/type.h"

namespace imu {

class AllanAcc {
 public:
  AllanAcc(std::string name, int maxCluster = 10000);
  ~AllanAcc();
  void PushRadPerSec(double data, double time);
  void PushDegreePerSec(double data, double time);
  void pushMPerSec2(double data, double time);
  void CalculateAllanVariance();

  std::vector<double> GetVariance() const;
  std::vector<double> GetDeviation();
  std::vector<double> GetTimestamp();
  std::vector<int> getFactors() const;
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
  std::vector<AccData> m_rawData;
  std::vector<double> m_thetas;
  int numCluster;
  int numFactors;
  std::vector<int> mFactors;

  std::vector<double> mVariance;
};
}  // namespace imu

#endif  // AllanAcc_H
