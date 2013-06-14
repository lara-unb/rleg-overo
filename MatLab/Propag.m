seizfunction XaVec=Propag(AlfaVec)
N=size(AlfaVec);
n=N(2);
for i=1:n
[Xv,Xa]=accelerations2(AlfaVec(i), 0, [0;0], 0, 1, [0;0], 0, 0, 0, [0;0]);
XaVec(:,i)=Xa;
end
