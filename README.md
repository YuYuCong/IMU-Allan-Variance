# IMU Allan Variance

**REFERENCES**

- [gaowenliang/imu_utils](https://github.com/gaowenliang/imu_utils)
- [rpng/kalibr_allan](https://github.com/rpng/kalibr_allan)
- N. El-Sheimy, H. Hou and X.
  Niu, "[Analysis and Modeling of Inertial Sensors Using Allan Variance](https://www.researchgate.net/publication/3094132_Analysis_and_Modeling_of_Inertial_Sensors_Using_Allan_Variance)"
  . in IEEE Transactions on Instrumentation and Measurement, vol. 57, no. 1, pp.
  140-149, Jan. 2008, doi: 10.1109/TIM.2007.908635.

The code of this project is mainly based
on [this repository](https://github.com/gaowenliang/imu_utils) and [this repository](https://github.com/mintar/imu_utils), thanks to the
work of [gaowenliang](https://github.com/gaowenliang), [mintar](https://github.com/mintar) et al.

### IMU Noise Values

Parameter | YAML element | Symbol | Units
--- | --- | --- | ---
Gyroscope "white noise" | `gyr_n` | <img src="https://latex.codecogs.com/svg.latex?{%5Csigma_g}"> | <img src="https://latex.codecogs.com/svg.latex?{%5Cfrac%7Brad%7D%7Bs%7D%5Cfrac%7B1%7D%7B%5Csqrt%7BHz%7D%7D}">
Accelerometer "white noise" | `acc_n` | <img src="https://latex.codecogs.com/svg.latex?{%5Csigma_a}"> | <img src="https://latex.codecogs.com/svg.latex?{%5Cfrac%7Bm%7D%7Bs^2%7D%5Cfrac%7B1%7D%7B%5Csqrt%7BHz%7D%7D}">
Gyroscope "bias Instability" | `gyr_w` | <img src="https://latex.codecogs.com/svg.latex?{%5Csigma_b_g}"> | <img src="http://latex.codecogs.com/svg.latex?\frac{rad}{s}&space;\sqrt{Hz}" title="\frac{rad}{s} \sqrt{Hz}" />
Accelerometer "bias Instability" | `acc_w` | <img src="https://latex.codecogs.com/svg.latex?{%5Csigma_b_a}"> | <img src="http://latex.codecogs.com/svg.latex?\frac{m}{s^2}&space;\sqrt{Hz}" title="\frac{m}{s^2} \sqrt{Hz}" />

* White noise is at tau=1;

* Bias Instability is around the minimum;

(according to technical report: [`Allan Variance: Noise Analysis for Gyroscopes`](http://cache.freescale.com/files/sensors/doc/app_note/AN5087.pdf "Allan Variance: Noise Analysis for Gyroscopes"))

### Requirements

1. Eigen3
2. Ceres
3. matplotlib

### Usage

1. 编译

```shell
mkdir build
cd build
cmake ..
make -j4
```

2. 准备 采样的数据

put your imu sampled data file "imu_reading.txt" at dir ./data/

3. run allan_variance analysis

```shell
cd build
./imu_allan_variance
```

4. result

在data文件夹下面可以看到数据:

- data_IMUxxxx_acc_t.txt: acc时间戳
- data_IMUxxxx_acc_x.txt: acc采样x轴
- data_IMUxxxx_acc_y.txt: acc采样y轴
- ...
- data_IMUxxxx_sim_acc_t.txt: acc拟合时间戳
- data_IMUxxxx_sim_acc_y.txt: acc拟合y轴曲线
- ...
- IMUxxxx_imu_param.txt: IMU噪声分析解结果

5. 绘制曲线

```shell
cd ./scripts
python3 ./draw_allan.py
```

<img src="scripts/allan_variance_of_gyro.png">
<img src="scripts/allan_variance_of_acc.png">
