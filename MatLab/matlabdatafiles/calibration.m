function [Param,resnorm] = calibration(m)
% p = [ x0, y0, z0, a, b, c ]
%global meas;
%meas=m;
%m=evalin('base','meastemp');
means=mean(m);
e = @(p) sqrt((m(:,1)-means(1)*m(:,1).^0).^2+(m(:,2)-means(2)*m(:,1).^0).^2+(m(:,3)-means(3)*m(:,1).^0).^2).*(m(:,1).^0-(m(:,1).^0)./(((m(:,1)-means(1)*m(:,1).^0).^2)/p(1)+((m(:,2)-means(2)*m(:,1).^0).^2)/p(2)+((m(:,3)-means(3)*m(:,1).^0).^2)/p(3)));
r=mean(sqrt((m(:,1)-means(1)*m(:,1).^0).^2+(m(:,2)-means(2)*m(:,2).^0).^2+(m(:,3)-means(3)*m(:,3).^0).^2))
%p0=[means(1);means(2);means(3);r;r;r]
p0=[r;r;r]
options=optimset('PlotFcns',@optimplotresnorm,'TolFun',1e-80, 'Display', 'iter');
[Param,resnorm]=lsqnonlin(e,p0,[],[],options);
%[Param,resnorm]=lsqnonlin(@eof,p0,[-200;-200;-200;-800;-800;-800],[200;200;200;800;800;800],options);
end

% function e = eof(p)
% meas=evalin('base','meastemp');
% 
% n=size(meas);
% n=n(1);
% 
% for k=1:n
%     e(k,1)=sqrt((meas(k,1)-p(1))^2+(meas(k,2)-p(2))^2+(meas(k,3)-p(3))^2)*(1-1/(((meas(k,1)-p(1))^2)/p(4)+((meas(k,2)-p(2))^2)/p(5)+((meas(k,3)-p(3))^2)/p(6)));
%     %e(k,1)=sqrt((meas(k,1)-p(1))^2+(meas(k,2)-p(2))^2+(meas(k,3)-p(3))^2)*(1-1/(((meas(k,1)-p(1))^2)/p(4)+((meas(k,2)-p(2))^2)/p(5)+((meas(k,3)-p(3))^2)/p(6)));
%     %e(k,1)=1-sqrt(((meas(k,1)-p(1))^2)/p(4)+((meas(k,2)-p(2))^2)/p(5)+((meas(k,3)-p(3))^2)/p(6));
% end
% 
% 
% end