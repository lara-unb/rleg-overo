function [Xv,Xa]=accelerations(Alfa, WAlfa, Vj, Wj, Fp, Fin, Tin, Fint, Tint, g)
%As posições serão dadas para o caso que a barra J está no eixo X e Ra = 0

%Parâmetros fixos do mecanismo
a=0.19783; Ke2=0.5; THe2=0;
b=0.12576; Kt=2; THt=17*pi/20;
c=0.08546; Ke1=0.5; THe1=0;
d=0.02383; Kj=-1; THj=0;

Mj= 0.3; Ij= 0.3;
Me1=0.2; Ie1=0.2;
Me2=0.4; Ie2=0.4;
Mt= 6; It= 6;

Kp=0.18032/0.19783; %BP/BC onde P é o ponto em que o pistão se fixa à barra E2
Mp=1;
Ip0=1; %momento de inércia para compressão máxima
Kip=0.2; %coeficiente linear do momento de inércia do pistão
Lcm0=3; %distância do ponto P ao centro de massa do pistão para compressão máxima
Klcm=0.1; %coeficiente linear do centro de massa do pistão: CMpistão = Lcm0 + Klcm*x, onde x é a diferença entre o comprimento do pistão (e) e seu comprimento mínimo (emin)
%Mpm=; %massa da parte móvel do pistão
%Mpf=; %massa da parte fixa do pistão (onde tem o fluido MR)
%Lpm=; % comprimento da parte móvel
%Lpf=5; % comprimento da parte fixa
anglemin=pi/12;
emin = (a^2+d^2-2*a*d*cos(anglemin))^(1/2);

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
rCT=-Kt*rotangle(rDC,THt);
rDT=rCT+rDC;
rBJ=-Kj*rotangle(rAB,THj);
rAJ=rBJ+rAB;

%Construção das equações das velocidades (Ax=B)
I2=[1 0; 0 1];
O2=zeros(2);

A=[ -I2 O2 O2 O2 O2 [0;0]               O2 [0;0]                O2 [0;0]            I2 [-rAJ(2);rAJ(1)] ;
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
Xv=A\B; %Xv = [Va, Vb, Vc, Vd, Ve1, We1, Ve2, We2, Vt, Wt, Vj, Wj]^t

%Construção das esquações das acelerações
We1=Xv(11);
We2=Xv(14);
Wt=Xv(17);
Wj=Xv(20);
%Wp=(Xv(2)-Xv(4)-We2*rPB(1))/rAP(1); %Wp x rAP = Va-Vb-(We2 x rPB) => Wp = (Va(2)-Vb(2)-We2*rPB(1))/rAP(1);

rAP=rAB-rPB;
eaAP=rAP/norm(rAP); %vetor unitário na direção axial do pistão
etAP=rotangle(eaAP,-pi/2); %vetor unitário na direção transversal ao pistão
F=Fp*eaAP;
Lcm=Lcm0+Klcm*(e-emin);
Ip=Ip0+Kip*(e-emin);
%Me1=0;Ie1=0; Me2=0; Ie2=0; Mj=0; Ij=0; %Mp=0;Ip=0;

Cross=[0 -1; 1 0];

A=[ -I2 O2 O2 O2 O2 [0;0]               O2 [0;0]        O2 [0;0]        I2  Cross*rAJ   O2 O2 O2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    O2 -I2 O2 O2 O2 [0;0]               O2 [0;0]        O2 [0;0]        I2  Cross*rBJ   O2 O2 O2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    -I2 O2 O2 O2 I2 Cross*rAE1  O2 [0;0]                O2 [0;0]        O2  [0;0]       O2 O2 O2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    O2 O2 O2 -I2 I2 Cross*rDE1  O2 [0;0]                O2 [0;0]        O2  [0;0]       O2 O2 O2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    O2 -I2 O2 O2 O2 [0;0]               I2 Cross*rBE2   O2 [0;0]        O2  [0;0]       O2 O2 O2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    O2 O2 -I2 O2 O2 [0;0]               I2 Cross*rCE2   O2 [0;0]        O2  [0;0]       O2 O2 O2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    O2 O2 O2 -I2 O2 [0;0]               O2 [0;0]        I2 Cross*rDT    O2  [0;0]       O2 O2 O2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    O2 O2 -I2 O2 O2 [0;0]               O2 [0;0]        I2 Cross*rCT    O2  [0;0]       O2 O2 O2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    O2 O2 O2 O2  O2 [0;0]               O2 [0;0]        O2 [0;0]        -Mj*I2 [0;0]    I2 I2 O2 O2    O2 [0;0] [0;0] etAP  [0;0];
    O2 O2 O2 O2  -Me1*I2 [0;0]          O2 [0;0]        O2 [0;0]        O2  [0;0]       -I2 O2 -I2 O2    O2 [0;0] [0;0] [0;0] [0;0];
    O2 O2 O2 O2  O2 [0;0]               -Me2*I2 [0;0]   O2 [0;0]        O2  [0;0]       O2 -I2 O2 -I2    O2 [0;0] [0;0] [0;0] -etAP;
    O2 O2 O2 O2  O2 [0;0]               O2 [0;0]        -Mt*I2 [0;0]    O2  [0;0]       O2 O2 I2 I2    O2 [0;0] [0;0] [0;0] [0;0];
    zeros(1,19)    -Ij                                                                  (Cross*rAJ)' (Cross*rBJ)' [0 0] [0 0]     [0 0] 0 0 (Cross*rAJ)'*etAP 0 ;
    zeros(1,10) -Ie1  zeros(1,9)                                                              -(Cross*rAE1)' [0 0] -(Cross*rDE1)' [0 0]       [0 0] 0 0 0 0;
    zeros(1,13) -Ie2  zeros(1,6)                                                              [0 0] -(Cross*rBE2)' [0 0] -(Cross*rCE2)'       [0 0] 0 0 0 -(Cross*rPE2)'*etAP  ;
    zeros(1,16) -It    zeros(1,3)                                                              [0 0] [0 0] (Cross*rDT)' (Cross*rCT)'            [0 0] 0 0 0 0;
    O2 O2 O2 O2  O2 [0;0]               I2 Cross*rPE2   O2 [0;0]            O2  [0;0]            O2 O2 O2 O2   -I2 [0;0] [0;0] [0;0] [0;0];  
    -etAP' zeros(1,26)                                                                                                [0 0] 1 -(e-Lcm) 0 0;
    zeros(1,28)                                                                                                      -etAP' 1 Lcm 0 0;
    zeros(1,30)                                                                                                             -Mp    0  -1 1;
    zeros(1,31)                                                                                                                   -Ip (e-Lcm) Lcm; ];
%inv(A)'
B=[ Wj^2*[rAJ;rBJ]; We1^2*[rAE1;rDE1]; We2^2*[rBE2;rCE2]; Wt^2*[rDT;rCT]; (-F-Fin-Mj*g); -Me1*g; F-Me2*g; -Mt*g-Fint; (rAJ(2)*F(1)-rAJ(1)*F(2)-Tin); 0; (-rPE2(2)*F(1)+rPE2(1)*F(2)); -Tint; We2^2*rPE2; 0; 0; -Mp*etAP'*g; 0 ];
%B=[ zeros(16,1); (-Mj*g); -Me1*g; -Me2*g; -Mt*g; 0; 0; 0; 0; 0; 0; 0; 0; -Mp*etAP'*g; 0 ];
Xa=A\B; % Xa = [Aa, Ab, Ac, Ad, Ae1, ALFe1, Ae2, ALFe2, At, ALFt, Aj, ALFj, Fje1, Fje2, Fte1, Fte2, Ap, Amt, ALFm, Fjp, Fpe2];

Rot=[0 1;-1 0];
CIR=[(Rot*rCB)';(Rot*rDA)']\[0;rAB'*Rot*rDA];
Atcir=[Xa(5);Xa(6)]+Xa(17)*[0 -1;1 0]*(CIR-rCB);
Ajcir=[Xa(3);Xa(4)]+Xa(20)*[0 -1;1 0]*(CIR);

    


function v=rotangle(u,angle)
M=[cos(angle) -sin(angle); sin(angle) cos(angle)];
v=M*u;



