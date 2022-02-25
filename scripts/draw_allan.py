import matplotlib.pyplot as plt


def ReadFromSimpleFile(file_path='./log.txt'):
    """
    @brief read data from log file
        file format: one num one line, annotation with '#' in line
    @input file path
    @return data in python list
    """
    data_list = []
    file = open(file_path)
    while True:
        line = file.readline()
        if not line:
            break
        elif '#' in line:
            continue
        else:
            data_list.append(float(line))
    return data_list


def VizAllanVariance(title="", all_data=[("label", [], [])]):
    """
    @brief viz Allan Variance data
    @input title
    @input all_data
        format [("label_1", x_1=[], y_1=[]), ("label_1", x_1=[], y_1=[]), ...]
    """
    plt.figure(figsize=(8, 6))
    for data_i in all_data:
        label = data_i[0]
        x = data_i[1]
        y = data_i[2]
        if ('sim' in label):
            plt.plot(x, y, label=label)
        else:
            plt.scatter(x, y, label=label, s=8)
        plt.loglog(basex=10, basey=10)
    plt.xlabel(r'$\tau[s]$')
    if ('gyr' in label):
        plt.ylabel(r'$\sigma(\tau)[deg/h]$')
        # or use data/3600 to set ylabel deg/s
        # plt.ylabel(r'$\sigma(\tau)[deg/s]$')
    elif ('acc' in label):
        plt.ylabel(r'$\sigma(\tau)[g]$')
    plt.grid()
    plt.legend()
    plt.title(title)
    plt.savefig("./" + title + ".png")
    plt.show()


def main():
    print("Allan Variance Plot.")

    # read timestamp
    gyro_time_list = ReadFromSimpleFile('../data/data_imu6050_gyr_t.txt')
    # read gyro allan data
    gyro_x_list = ReadFromSimpleFile('../data/data_imu6050_gyr_x.txt')
    gyro_y_list = ReadFromSimpleFile('../data/data_imu6050_gyr_y.txt')
    gyro_z_list = ReadFromSimpleFile('../data/data_imu6050_gyr_z.txt')
    # read timestamp
    acc_time_list = ReadFromSimpleFile('../data/data_imu6050_acc_t.txt')
    # read acc allan data
    acc_x_list = ReadFromSimpleFile('../data/data_imu6050_acc_x.txt')
    acc_y_list = ReadFromSimpleFile('../data/data_imu6050_acc_y.txt')
    acc_z_list = ReadFromSimpleFile('../data/data_imu6050_acc_z.txt')

    # read sim gyro allan data
    sim_gyro_x_list = ReadFromSimpleFile('../data/data_imu6050_sim_gyr_x.txt')
    sim_gyro_y_list = ReadFromSimpleFile('../data/data_imu6050_sim_gyr_y.txt')
    sim_gyro_z_list = ReadFromSimpleFile('../data/data_imu6050_sim_gyr_z.txt')

    # read sim acc allan data
    sim_acc_x_list = ReadFromSimpleFile('../data/data_imu6050_sim_acc_x.txt')
    sim_acc_y_list = ReadFromSimpleFile('../data/data_imu6050_sim_acc_y.txt')
    sim_acc_z_list = ReadFromSimpleFile('../data/data_imu6050_sim_acc_z.txt')

    VizAllanVariance(title="allan_variance_of_gyro",
                     all_data=[("gyro_x", gyro_time_list, gyro_x_list),
                               ("gyro_y", gyro_time_list, gyro_y_list),
                               ("gyro_z", gyro_time_list, gyro_z_list),
                               ("sim_gyro_x", gyro_time_list, sim_gyro_x_list),
                               ("sim_gyro_y", gyro_time_list, sim_gyro_y_list),
                               ("sim_gyro_z", gyro_time_list,
                                sim_gyro_z_list), ])

    VizAllanVariance(title="allan_variance_of_acc",
                     all_data=[("acc_x", acc_time_list, acc_x_list),
                               ("acc_y", acc_time_list, acc_y_list),
                               ("acc_z", acc_time_list, acc_z_list),
                               ("sim_acc_x", acc_time_list, sim_acc_x_list),
                               ("sim_acc_y", acc_time_list, sim_acc_y_list),
                               ("sim_acc_z", acc_time_list, sim_acc_z_list), ])


if __name__ == "__main__":
    main()
