#include "./acc_lib/allan_acc.h"
#include "./acc_lib/fitallan_acc.h"
#include "./gyr_lib/allan_gyr.h"
#include "./gyr_lib/fitallan_gyr.h"
#include "type.h"

// #include <boost/filesystem.hpp>
#include <ctime>
#include <fstream>
#include <iomanip>  // for std::setprecision
#include <iostream>
#include <mutex>
#include <queue>

imu::AllanGyr *gyr_x;
imu::AllanGyr *gyr_y;
imu::AllanGyr *gyr_z;
imu::AllanAcc *acc_x;
imu::AllanAcc *acc_y;
imu::AllanAcc *acc_z;

static inline std::vector<std::string> SplitString(const std::string &str,
                                                   const std::string &pattern) {
  std::vector<std::string> res;
  if (str == "") {
    return res;
  }
  std::string strs = str + pattern;
  size_t pos = strs.find(pattern);

  while (pos != strs.npos) {
    std::string temp = strs.substr(0, pos);
    res.push_back(temp);
    strs = strs.substr(pos + 1, strs.size());
    pos = strs.find(pattern);
  }
  return res;
}

/**
 * @brief convert standard time in string to timestamp
 * @param str_time time in format "yyyy-mm-dd hh:MM:ss"
 * @return timestamp in second
 */
static inline time_t StandardTimeToTimestampSecond(
    const std::string &time_string) {
  struct tm tm;
  memset(&tm, 0, sizeof(tm));
  sscanf(time_string.c_str(), "%d-%d-%d %d:%d:%d", &tm.tm_year, &tm.tm_mon,
         &tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
  tm.tm_year -= 1900;
  tm.tm_mon--;
  return mktime(&tm);
}

/**
 * @brief convert standard time in string to timestamp
 * @param str_time time in format "yyyy-mm-dd hh:MM:ss.mmmmmm"
 * @return timestamp in milli second
 */
static inline double StandardTimeToTimestampMilliSecond(
    const std::string &time_string) {
  double second = StandardTimeToTimestampSecond(time_string.substr(0, 19));
  double millisecond = std::stod((time_string.substr(20, 6)).c_str());
  double timestamp = second * 1000 + millisecond;
  return timestamp;
}

/**
 * @brief read imu data from file
 * @param file path to imu data file
 *  file format eg:
E0928 09:53:55.873848 13488 imu_data.cpp:162]
imu:0,0.00492204,-0.00133028,0.00186239,0.401953,-0.162695,9.67559
 timestamp,gx,gy,gz,ax,ay,az
 */
bool ReadImuDataGlog(const std::string &file) {
  //  if (!boost::filesystem::exists(file)) {
  //    std::cerr << "Path not exist, please check:" << file << std::endl;
  //    return false;
  //  }

  std::ifstream infile(file);
  std::string line;
  while (std::getline(infile, line)) {
    if (line.find("#") != std::string::npos) {
      continue;
    }
    auto substrs = SplitString(line, ",");
    if (substrs.size() != 7) {
      std::cerr << "Log format incorrect," << line << std::endl;
      return false;
    }

    // Glog timestamp format: E0928 09:53:55.872972
    std::string yy = "2021";
    std::string mm = SplitString(line, " ")[0].substr(1, 2);
    std::string dd = SplitString(line, " ")[0].substr(3, 2);
    std::string hhmmss_mmmmmm = SplitString(line, " ")[1];
    std::string timestamp_string =
        yy + "-" + mm + "-" + dd + " " + hhmmss_mmmmmm;
    double timestamp =
        StandardTimeToTimestampMilliSecond(timestamp_string) / 1000.0;

    ImuReading imu_reading(timestamp, stod(substrs[1]), stod(substrs[2]),
                           stod(substrs[3]), stod(substrs[4]), stod(substrs[5]),
                           stod(substrs[6]));
    gyr_x->PushRadPerSec(imu_reading.gyro.x, imu_reading.timestamp);
    gyr_y->PushRadPerSec(imu_reading.gyro.y, imu_reading.timestamp);
    gyr_z->PushRadPerSec(imu_reading.gyro.z, imu_reading.timestamp);
    acc_x->pushMPerSec2(imu_reading.acc.x, imu_reading.timestamp);
    acc_y->pushMPerSec2(imu_reading.acc.y, imu_reading.timestamp);
    acc_z->pushMPerSec2(imu_reading.acc.z, imu_reading.timestamp);
  }
  return true;
}

/**
 * @brief read imu data from file
 * @param file path to imu data file
*  file format eg:
1645962084.456809 -1.5219385 1.5444301 9.8459334 -0.0266748 -0.0209981 0.6608164
 timestamp,gx,gy,gz,ax,ay,az
 */
bool ReadImuDataSimpleFormat(const std::string &file) {
  //  if (!boost::filesystem::exists(file)) {
  //    std::cerr << "Path not exist, please check:" << file << std::endl;
  //    return false;
  //  }

  std::ifstream infile(file);
  std::string line;
  while (std::getline(infile, line)) {
    if (line.find("#") != std::string::npos) {
      continue;
    }
    auto substrs = SplitString(line, " ");
    if (substrs.size() != 7) {
      std::cerr << "Log format incorrect," << substrs.size() << std::endl;
      for (const auto &i : substrs) {
        std::cout << i << " ";
      }
      std::cout << std::endl;
      return false;
    }

    const std::string timestamp_string = substrs[0];
    double timestamp = stod(timestamp_string);
    ImuReading imu_reading(timestamp, stod(substrs[1]), stod(substrs[2]),
                           stod(substrs[3]), stod(substrs[4]), stod(substrs[5]),
                           stod(substrs[6]));
    gyr_x->PushRadPerSec(imu_reading.gyro.x, imu_reading.timestamp);
    gyr_y->PushRadPerSec(imu_reading.gyro.y, imu_reading.timestamp);
    gyr_z->PushRadPerSec(imu_reading.gyro.z, imu_reading.timestamp);
    acc_x->pushMPerSec2(imu_reading.acc.x, imu_reading.timestamp);
    acc_y->pushMPerSec2(imu_reading.acc.y, imu_reading.timestamp);
    acc_z->pushMPerSec2(imu_reading.acc.z, imu_reading.timestamp);
  }
  return true;
}

/**
 * @brief write data
 * @param
 */
void WriteDataOneSingleAxis(const std::string &data_save_path,
                            const std::string &data_name,
                            const std::vector<double> &timestamp,
                            const std::vector<double> &write_data) {
  std::ofstream out_t;
  std::ofstream out_x;
  out_t.open(data_save_path + "data_" + data_name + "_t.txt", std::ios::trunc);
  out_x.open(data_save_path + "data_" + data_name + "_x.txt", std::ios::trunc);
  out_t << std::setprecision(10);
  out_x << std::setprecision(10);
  for (unsigned int index = 0; index < timestamp.size(); ++index) {
    out_t << timestamp[index] << '\n';
    out_x << write_data[index] << '\n';
  }
  out_t.close();
  out_x.close();
}

/**
 * @brief write data
 * @param
 */
void WriteDataAllThreeAxis(const std::string &data_save_path,
                           const std::string &data_name,
                           const std::vector<double> &timestamp,
                           const std::vector<double> &write_data_x,
                           const std::vector<double> &write_data_y,
                           const std::vector<double> &write_data_z) {
  std::ofstream out_t;
  std::ofstream out_x;
  std::ofstream out_y;
  std::ofstream out_z;
  out_t.open(data_save_path + "data_" + data_name + "_t.txt", std::ios::trunc);
  out_x.open(data_save_path + "data_" + data_name + "_x.txt", std::ios::trunc);
  out_y.open(data_save_path + "data_" + data_name + "_y.txt", std::ios::trunc);
  out_z.open(data_save_path + "data_" + data_name + "_z.txt", std::ios::trunc);
  out_t << std::setprecision(10);
  out_x << std::setprecision(10);
  out_y << std::setprecision(10);
  out_z << std::setprecision(10);

  for (size_t index = 0; index < timestamp.size(); ++index) {
    out_t << timestamp[index] << '\n';
    out_x << write_data_x[index] << '\n';
    out_y << write_data_y[index] << '\n';
    out_z << write_data_z[index] << '\n';
  }

  out_t.close();
  out_x.close();
  out_y.close();
  out_z.close();
}

void WriteResult(const std::string &data_path, const std::string &sensor_name,
                 const imu::FitAllanGyr &gyr_x, const imu::FitAllanGyr &gyr_y,
                 const imu::FitAllanGyr &gyr_z, const imu::FitAllanAcc &acc_x,
                 const imu::FitAllanAcc &acc_y, const imu::FitAllanAcc &acc_z) {
  std::ofstream out_file;
  out_file.open(data_path + "imu_param_" + sensor_name + ".txt",
                std::ios::trunc);

  out_file << "sensor_type: "
           << "IMU" << '\n';
  out_file << "name: " << sensor_name << '\n';

  out_file << "Gyro:\n";
  out_file << "\tunit:"
           << "gyr_n: rad / sqrt(s), gyr_w: rad / s^2 / sqrt(Hz)\n";

  out_file << "\tavg-axis:\n";
  out_file << "\t\tgyr_n: "
           << (gyr_x.getN() + gyr_y.getN() + gyr_z.getN()) / 3.0 << '\n';
  out_file << "\t\tgyr_w: "
           << (gyr_x.getK() + gyr_y.getK() + gyr_z.getK()) / 3.0 << '\n';

  out_file << "\tx-axis:\n";
  out_file << "\t\tgyr_n: " << gyr_x.getN() << '\n';
  out_file << "\t\tgyr_w: " << gyr_x.getK() << '\n';

  out_file << "\ty-axis:\n";
  out_file << "\t\tgyr_n: " << gyr_y.getN() << '\n';
  out_file << "\t\tgyr_w: " << gyr_y.getK() << '\n';

  out_file << "\tz-axis: \n";
  out_file << "\t\tgyr_n: " << gyr_z.getN() << '\n';
  out_file << "\t\tgyr_w: " << gyr_z.getK() << '\n';

  out_file << "Acc:\n";
  out_file << "\tunit:"
           << "acc_n: m / s^2 / sqrt(Hz), acc_w: m / s^3 / sqrt(Hz)\n";

  out_file << "\tavg-axis\n";
  out_file << "\t\tacc_n: " << (acc_x.getN() + acc_y.getN() + acc_z.getN()) / 3
           << '\n';
  out_file << "\t\tacc_w: " << (acc_x.getK() + acc_y.getK() + acc_z.getK()) / 3
           << '\n';

  out_file << "\tx-axis:\n";
  out_file << "\t\tacc_n: " << acc_x.getN() << '\n';
  out_file << "\t\tacc_w: " << acc_x.getK() << '\n';

  out_file << "\ty-axis:\n";
  out_file << "\t\tacc_n: " << acc_y.getN() << '\n';
  out_file << "\t\tacc_w: " << acc_y.getK() << '\n';

  out_file << "\tz-axis:\n";
  out_file << "\t\tacc_n: " << acc_z.getN() << '\n';
  out_file << "\t\tacc_w: " << acc_z.getK() << '\n';

  out_file.close();
}

int main(int argc, char **argv) {
  std::string imu_data_path = "../data/imu_reading.txt";
  std::string imu_name = "IMUxxxx";
  std::string data_save_path = "../data/";
  int max_cluster = 100;  // todo(congyu) read from config

  gyr_x = new imu::AllanGyr("gyr x", max_cluster);
  gyr_y = new imu::AllanGyr("gyr y", max_cluster);
  gyr_z = new imu::AllanGyr("gyr z", max_cluster);
  acc_x = new imu::AllanAcc("acc x", max_cluster);
  acc_y = new imu::AllanAcc("acc y", max_cluster);
  acc_z = new imu::AllanAcc("acc z", max_cluster);

  std::cout << "reading imu data..." << std::endl;
  //  if (!ReadImuData(imu_data_path)) {
  //    return 0;
  //  }

  imu_name = "imuSimulation";
  imu_data_path = "../data/imu_noise_data_sim.txt";
  if (!ReadImuDataSimpleFormat(imu_data_path)) {
    return 0;
  }

  /// calc allan variance
  std::cout << "====================== gyro ========================"
            << std::endl;
  gyr_x->CalculateAllanVariance();
  std::vector<double> gyro_v_x = gyr_x->GetVariance();
  std::vector<double> gyro_d_x = gyr_x->GetDeviation();
  std::vector<double> gyro_ts_x = gyr_x->GetTimestamp();

  gyr_y->CalculateAllanVariance();
  std::vector<double> gyro_v_y = gyr_y->GetVariance();
  std::vector<double> gyro_d_y = gyr_y->GetDeviation();
  std::vector<double> gyro_ts_y = gyr_y->GetTimestamp();

  gyr_z->CalculateAllanVariance();
  std::vector<double> gyro_v_z = gyr_z->GetVariance();
  std::vector<double> gyro_d_z = gyr_z->GetDeviation();
  std::vector<double> gyro_ts_z = gyr_z->GetTimestamp();

  /// fit allan result
  std::cout << "fitting gyro x..." << std::endl;
  imu::FitAllanGyr fit_gyr_x(gyro_v_x, gyro_ts_x, gyr_x->GetFrequency());
  std::cout << "  bias " << gyr_x->GetAvgValue() << " rad/s" << std::endl;
  std::cout << "-------------------" << std::endl;

  std::cout << "fitting gyro y..." << std::endl;
  imu::FitAllanGyr fit_gyr_y(gyro_v_y, gyro_ts_y, gyr_y->GetFrequency());
  std::cout << "  bias " << gyr_y->GetAvgValue() << " rad/s" << std::endl;
  std::cout << "-------------------" << std::endl;

  std::cout << "fitting gyro z..." << std::endl;
  imu::FitAllanGyr fit_gyr_z(gyro_v_z, gyro_ts_z, gyr_z->GetFrequency());
  std::cout << "  bias " << gyr_z->GetAvgValue() << " rad/s" << std::endl;
  std::cout << "-------------------" << std::endl;

  std::vector<double> gyro_sim_d_x = fit_gyr_x.CalculateSimDeviation(gyro_ts_x);
  std::vector<double> gyro_sim_d_y = fit_gyr_y.CalculateSimDeviation(gyro_ts_y);
  std::vector<double> gyro_sim_d_z = fit_gyr_z.CalculateSimDeviation(gyro_ts_z);

  std::cout << "====================== acc ======================" << std::endl;
  acc_x->CalculateAllanVariance();
  std::vector<double> acc_v_x = acc_x->GetVariance();
  std::vector<double> acc_d_x = acc_x->GetDeviation();
  std::vector<double> acc_ts_x = acc_x->GetTimestamp();

  acc_y->CalculateAllanVariance();
  std::vector<double> acc_v_y = acc_y->GetVariance();
  std::vector<double> acc_d_y = acc_y->GetDeviation();
  std::vector<double> acc_ts_y = acc_y->GetTimestamp();

  acc_z->CalculateAllanVariance();
  std::vector<double> acc_v_z = acc_z->GetVariance();
  std::vector<double> acc_d_z = acc_z->GetDeviation();
  std::vector<double> acc_ts_z = acc_z->GetTimestamp();

  std::cout << "fitting acc x..." << std::endl;
  imu::FitAllanAcc fit_acc_x(acc_v_x, acc_ts_x, acc_x->GetFrequency());
  std::cout << "-------------------" << std::endl;

  std::cout << "fitting acc y..." << std::endl;
  imu::FitAllanAcc fit_acc_y(acc_v_y, acc_ts_y, acc_y->GetFrequency());
  std::cout << "-------------------" << std::endl;

  std::cout << "fitting acc z..." << std::endl;
  imu::FitAllanAcc fit_acc_z(acc_v_z, acc_ts_z, acc_z->GetFrequency());
  std::cout << "-------------------" << std::endl;

  std::vector<double> acc_sim_d_x = fit_acc_x.CalculateSimDeviation(acc_ts_x);
  std::vector<double> acc_sim_d_y = fit_acc_y.CalculateSimDeviation(acc_ts_x);
  std::vector<double> acc_sim_d_z = fit_acc_z.CalculateSimDeviation(acc_ts_x);

  /// log to viz
  WriteDataAllThreeAxis(data_save_path, imu_name + "_fitting_gyr", gyro_ts_x,
                        gyro_sim_d_x, gyro_sim_d_y, gyro_sim_d_z);
  WriteDataAllThreeAxis(data_save_path, imu_name + "_gyr", gyro_ts_x, gyro_d_x,
                        gyro_d_y, gyro_d_z);

  WriteDataAllThreeAxis(data_save_path, imu_name + "_fitting_acc", acc_ts_x,
                        acc_sim_d_x, acc_sim_d_y, acc_sim_d_z);
  WriteDataAllThreeAxis(data_save_path, imu_name + "_acc", acc_ts_x, acc_d_x,
                        acc_d_y, acc_d_z);

  /// write result
  WriteResult(data_save_path, imu_name, fit_gyr_x, fit_gyr_y, fit_gyr_z,
              fit_acc_x, fit_acc_y, fit_acc_z);

  return 0;
}
