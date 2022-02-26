#ifndef TYPE_H
#define TYPE_H
#include <ctime>
#include <iostream>

class Data {
 public:
  Data() : v(0.0), t(0.0) {}
  Data(double data, double time) : v(data), t(time) {}

  double v;
  double t;
};

class AccData {
 public:
  AccData() : a(0.0), t(0.0) {}
  AccData(double data, double time) : a(data), t(time) {}

  double a;
  double t;
};

class GyrData {
 public:
  GyrData() : w(0.0), t(0.0) {}
  GyrData(double data, double time) : w(data), t(time) {}

  double w;
  double t;
};

struct ImuReading {
 public:
  struct Acc {
    double x = 0, y = 0, z = 0;
    Acc() = default;
    Acc(const double x, const double y, const double z) : x(x), y(y), z(z) {}
  };
  struct Gyro {
    double x = 0, y = 0, z = 0;
    Gyro() = default;
    Gyro(const double x, const double y, const double z) : x(x), y(y), z(z) {}
  };
  ImuReading() = default;
  ImuReading(const double timestamp, const double gx, const double gy,
             const double gz, const double ax, const double ay, const double az)
      : timestamp(timestamp) {
    gyro = Gyro(gx, gy, gz);
    acc = Acc(ax, ay, az);
  }

  double timestamp = 0; // second
  Gyro gyro;
  Acc acc;
};

#endif  // TYPE_H
