function [Xv,Xa]=positions(Alfa, WAlfa, Vj, Wj, Fp, Fin, Tin)
%As posições serão dadas para o caso que a barra J está no eixo X e Ra = 0

%Parâmetros fixos do mecanismo
a=10; Ke2=0.5; THe2=0;
b=8; Kt=2; THt=9*pi/20;
c=3; Ke1=0.5; THe1=0;
d=1; Kj=-1; THj=0;

Mj= 0.3; Ij= 0.3;
Me1=0.2; Ie1=0.2;
Me2=0.4; Ie2=0.4;
Mt= 6; It= 6;

Kp=0.8; %BP/BC onde P é o ponto em que o pistão se fixa à barra E2

%Cálculo dos ângulos de cada braço em relação ao de referência (J)
e = (a^2+d^2-2*a*d*cos(Alfa))^(1/2);
beta = acos((e^2+d^2-a^2)/(2*e*d));
psi = acos((e^2+b^2-c^2)/(2*e*b));

ThetaT = - beta + psi;
rAB=rotangle([d;0],0);
rCB=rotangle([a;0],Alfa);
rDC=rotangle([b;0],ThetaT);
rDA=rCB+rDC-rAB;
rPB=Kp*rCB;
%temp=cart2pol(rAD(1),rAD(2));
%ThetaE2=temp(1)

%Cálculo dos vetores de braço para cada centro de massa
rAE1=-Ke1*rotangle(rDA,THe1);
rDE1=rAE1+rDA;
rBE2=-Ke2*rotangle(rCB,THe2);
rCE2=rBE2+rCB;
rPE2=rBE2+rPB;
rDT=-Kt*rotangle(rDC,THt);
rCT=rDT+rDC;
rAJ=-Kj*rotangle(rAB,THj);
rBJ=rAJ+rAB;

%Construção das equações das velocidades (Ax=B)
I2=[1 0; 0 1];
O2=zeros(2);

A=[-I2 O2 O2 O2 O2 [0;0]               O2 [0;0]                O2 [0;0]            I2 [-rAJ(2);rAJ(1)] ;
    O2 -I2 O2 O2 O2 [0;0]               O2 [0;0]                O2 [0;0]            I2 [-rBJ(2);rBJ(1)] ;
    -I2 O2 O2 O2 I2 [-rAE1(2);rAE1(1)]  O2 [0;0]                O2 [0;0]            O2 [0;0];
    O2 O2 O2 -I2 I2 [-rDE1(2);rDE1(1)]  O2 [0;0]                O2 [0;0]            O2 [0;0];
    O2 -I2 O2 O2 O2 [0;0]               I2 [-rBE2(2);rBE2(1)]   O2 [0;0]            O2 [0;0];
    O2 O2 -I2 O2 O2 [0;0]               I2 [-rCE2(2);rCE2(1)]   O2 [0;0]            O2 [0;0];
    O2 O2 O2 -I2 O2 [0;0]               O2 [0;0]                I2 [-rDT(2);rDT(1)] O2 [0;0];
    O2 O2 -I2 O2 O2 [0;0]               O2 [0;0]                I2 [-rCT(2);rCT(1)] O2 [0;0];
    [0,0] [0,0] [0,0] [0,0] [0,0] 0 [0,0] -1 [0,0] 0 [0,0] 1;
    O2 O2 O2 O2 O2 [0;0] O2 [0;0] O2 [0;0] I2 [0;0] ;
    [0,0] [0,0] [0,0] [0,0] [0,0] 0 [0,0] 0 [0,0] 0 [0,0] 1; ];  
B=[zeros(16,1);WAlfa;Vj;Wj];
Xv=inv(A)*B; %Xv = [Va, Vb, Vc, Vd, Ve1, We1, Ve2, We2, Vt, Wt, Vj, Wj]^t

%Construção das esquações das acelerações
We1=Xv(11);
We2=Xv(14);
Wt=Xv(17);
Wj=Xv(20);
F=Fp*(rAB-rPB)/norm(rAB-rPB);


A=[ -I2 O2 O2 O2 O2 [0;0]               O2 [0;0]                O2 [0;0]            I2  [-rAJ(2);rAJ(1)] O2 O2 O2 O2;
    O2 -I2 O2 O2 O2 [0;0]               O2 [0;0]                O2 [0;0]            I2  [-rBJ(2);rBJ(1)] O2 O2 O2 O2;
    -I2 O2 O2 O2 I2 [-rAE1(2);rAE1(1)]  O2 [0;0]                O2 [0;0]            O2  [0;0]            O2 O2 O2 O2;
    O2 O2 O2 -I2 I2 [-rDE1(2);rDE1(1)]  O2 [0;0]                O2 [0;0]            O2  [0;0]            O2 O2 O2 O2;
    O2 -I2 O2 O2 O2 [0;0]               I2 [-rBE2(2);rBE2(1)]   O2 [0;0]            O2  [0;0]            O2 O2 O2 O2;
    O2 O2 -I2 O2 O2 [0;0]               I2 [-rCE2(2);rCE2(1)]   O2 [0;0]            O2  [0;0]            O2 O2 O2 O2;
    O2 O2 O2 -I2 O2 [0;0]               O2 [0;0]                I2 [-rDT(2);rDT(1)] O2  [0;0]            O2 O2 O2 O2;
    O2 O2 -I2 O2 O2 [0;0]               O2 [0;0]                I2 [-rCT(2);rCT(1)] O2  [0;0]            O2 O2 O2 O2;
    O2 O2 O2 O2  O2 [0;0]               O2 [0;0]                O2 [0;0]         -Mj*I2 [0;0]            I2 I2 O2 O2;
    O2 O2 O2 O2  -Me1*I2 [0;0]          O2 [0;0]                O2 [0;0]            O2  [0;0]          -I2 O2 -I2 O2;
    O2 O2 O2 O2  O2 [0;0]               -Me2*I2 [0;0]           O2 [0;0]            O2  [0;0]          O2 -I2 O2 -I2;
    O2 O2 O2 O2  O2 [0;0]               O2 [0;0]                -Mt*I2 [0;0]        O2  [0;0]            O2 O2 I2 I2;
    zeros(1,19)    -Ij                                                                                [-rAJ(2) rAJ(1)] [-rBJ(2) rBJ(1)] [0 0] [0 0];
    zeros(1,10)  -Ie1  zeros(1,9)                                                             -[-rAE1(2) rAE1(1)] [0 0] -[-rDE1(2) rDE1(1)] [0 0];
    zeros(1,13) -Ie2  zeros(1,6)                                                              [0 0] -[-rBE2(2) rBE2(1)] [0 0] -[-rCE2(2) rCE2(1)];
    zeros(1,16) -It    zeros(1,3)                                                              [0 0] [0 0] [-rDT(2) rDT(1)] [-rCT(2) rCT(1)]; ];
B=[ Wj^2*[rAJ;rBJ]; We1^2*[rAE1;rDE1]; We2^2*[rBE2;rCE2]; Wt^2*[rDT;rCT]; (-F-Fin); [0;0]; F; [0;0]; (rAJ(2)*F(1)-rAJ(1)*F(2)-Tin); 0; (-rPE2(2)*F(1)+rPE2(1)*F(2)); 0 ];
Xa=inv(A)*B;
    


function v=rotangle(u,angle)
M=[cos(angle) -sin(angle); sin(angle) cos(angle)];
v=M*u;



