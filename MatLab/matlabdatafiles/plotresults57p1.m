figure(1);
plot3(r57p1_calib(:,1),r57p1_calib(:,2),r57p1_calib(:,3));
hold on;
plot3(r57p1_g(:,1),r57p1_g(:,2),r57p1_g(:,3),'r');
figure(2);
subplot(311); plot(t57,r57p1_calib(:,1));
subplot(312); plot(t57,r57p1_calib(:,2));
subplot(313); plot(t57,r57p1_calib(:,3));
figure(3);
subplot(311); plot(t57,r57p1_g(:,1),'r');
subplot(312); plot(t57,r57p1_g(:,2),'r');
subplot(313); plot(t57,r57p1_g(:,3),'r');