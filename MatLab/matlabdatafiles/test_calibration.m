function [r, r_calib, v, v_calib, meas_calib,r_calib_g,v_calib_g, acc, r_g]= test_calibration(t,meas,p)
% p = [bias_x, bias_y, bias_z, fs_x, fs_y, fs_z];
%orientation of the accelerometer: all g in z-axis
G=9.78; % m/s²
n=size(meas,1);
meas_calib=[ (meas(:,1)-p(1)*ones(n,1))/p(4) (meas(:,2)-p(2)*ones(n,1))/p(5) (meas(:,3)-p(3)*ones(n,1))/p(6) ];
%meas_calib=meas_calib-[dir(1)*ones(n,1) dir(2)*ones(n,1) dir(3)*ones(n,1)];

v=[integration([t,meas(:,1)],0) integration([t,meas(:,2)],0) integration([t,meas(:,3)],0) ];
v_calib=[integration([t,meas_calib(:,1)],0) integration([t,meas_calib(:,2)],0) integration([t,meas_calib(:,3)],0) ];

r=[integration([t,v(:,1)],0) integration([t,v(:,2)],0) integration([t,v(:,3)],0) ];
r_calib=[integration([t,v_calib(:,1)],0) integration([t,v_calib(:,2)],0) integration([t,v_calib(:,3)],0) ];

%meas_calib_g=meas_calib;
v_calib_g=v_calib;
r_calib_g=r_calib;
dir=r_calib(n,:)/norm(r_calib(n,:));
dir_raw=r(n,:)/norm(r(n,:))

acc=meas*(G/250)-[dir_raw(1)*ones(n,1) dir_raw(2)*ones(n,1) dir_raw(3)*ones(n,1)]*G;
v_g=[integration([t,acc(:,1)],0) integration([t,acc(:,2)],0) integration([t,acc(:,3)],0) ];
r_g=[integration([t,v_g(:,1)],0) integration([t,v_g(:,2)],0) integration([t,v_g(:,3)],0) ];

meas_calib=(meas_calib-[dir(1)*ones(n,1) dir(2)*ones(n,1) dir(3)*ones(n,1)])*G;
v_calib=[integration([t,meas_calib(:,1)],0) integration([t,meas_calib(:,2)],0) integration([t,meas_calib(:,3)],0) ];
r_calib=[integration([t,v_calib(:,1)],0) integration([t,v_calib(:,2)],0) integration([t,v_calib(:,3)],0) ];

end



function F = integration(f,F0)
% f = [x, y]
% F0 = initial value of the integral of f
n=size(f,1);
F=zeros(n,1);
for i=2:n
    F(i)=F(i-1)+((f(i,2)+f(i-1,2))/2)*(f(i,1)-f(i-1,1));
end

end