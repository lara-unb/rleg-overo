% IMU with a constante angular velocity in many directions 
meas=[imu_accelerometer_x_raw imu_accelerometer_y_raw,imu_accelerometer_z_raw,imu_magnetometer_x_raw,imu_magnetometer_y_raw,imu_magnetometer_z_raw];
p=zeros(6,2);
real_meas=zeros(size(meas));
n=size(meas);
n=n(1);
% for i=1:2 %for each sensor (accelerometer and magnetometer)
%     meastemp=meas(:,(3*i-2):(3*i));
%     %save tempmeas.mat meastemp;
% 
%     [p(:,i),res(i)] = calibration();
%     for k=1:3 %for each axis
%     real_meas(:,3*(i-1)+k)=(meas(:,3*(i-1)+k)-p(k,i)*ones(n,1))/p(3+k);
%     end
% end
for i=1:2 %for each sensor (accelerometer and magnetometer)
    meastemp=meas(:,(3*i-2):(3*i));
    %save tempmeas.mat meastemp;

    [centers,rads,evecs,pars] = ellipsoid_fit(meastemp);
    p(:,i)=[centers;rads];
    for k=1:3 %for each axis
    real_meas(:,3*(i-1)+k)=(meas(:,3*(i-1)+k)-p(k,i)*ones(n,1))/p(3+k);
    end
end
%close tempmeas.mat;