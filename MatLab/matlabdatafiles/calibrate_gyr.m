% IMU with no angular velocity

meas_gyr_raw=[imu_gyrometer_x_raw,imu_gyromter_y_raw,imu_gyrometer_z_raw];
n=size(meas_gyr_raw);
n=n(1);
means=mean(meas_gyr);
meas_gyr=meas_gyr_raw-[ones(n,1)*means(1),ones(n,1)*means(2),ones(n,1)*means(3)];