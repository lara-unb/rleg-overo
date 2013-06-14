function ThetaVec=Theta(AlfaVec)
%As posições serão dadas para o caso que a barra J está no eixo X e Ra = 0

%Parâmetros fixos do mecanismo
a=10;
b=8;
c=3;
d=1;
N=size(AlfaVec);
n=N(2);
%Cálculo dos ângulos de cada braço em relação ao de referência (J)
for i=1:n
    Alfa=AlfaVec(i);
    e = (a^2+d^2-2*a*d*cos(Alfa))^(1/2);
    beta = acos((e^2+d^2-a^2)/(2*e*d));
    psi = acos((e^2+b^2-c^2)/(2*e*b));
ThetaVec(i) = - beta + psi;
end
