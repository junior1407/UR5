%parâmetros:
clc;
close all;
clear all;
a1 = 0.40;
a2 = 0.25;
d1= 0.195;
d4=0;

syms theta1 real;
syms theta2 real;
syms theta4 real;
syms d3 real;
syms d4 real;
syms d1 real;
syms a1 real;
syms a2 real;
%   a alpha   d theta

%   a alpha   d theta
DH = [a1    0    d1    theta1;
    a2    pi    0    theta2 ;
    0     0    d3        0 ;
    0     0    d4     theta4 ];

GD=size(DH);
GDL=GD(1,1);

for x=1:GDL
    
    a=DH(x,1);%extrai parametros
    alpha=DH(x,2);
    d=DH(x,3);
    %theta=DH(x,4);

Atemp = [cos(DH(x,4))   -sin(DH(x,4))*cos(alpha)   sin(DH(x,4))*sin(alpha)    a*cos(DH(x,4));
         sin(DH(x,4))    cos(DH(x,4))*cos(alpha)   -cos(DH(x,4))*sin(alpha)   a*sin(DH(x,4));
          0             sin(alpha)              cos(alpha)               d;
          0                 0                       0                    1];

 A(:,:,x)=Atemp;%forma as matrizes A01, A12, A23...An-1 n
      
end

%Matriz do Link 1
A(:,:, 1);



%% Calculo da matriz de transformação:
 T(:,:,1)= A(:,:,1);
 
 for x=2:1:GDL
 
     Ttemp = A(:,:,x);
     
     T(:,:,x)= T(:,:,(x-1))*Ttemp;
      T(:,:,x)= simplify(T(:,:,x));
 
 end
 
 
I1 = [97,0,0;0,100,0;0,0,4]*10^(-3);
I2 = [227,0,0;0,182,0;0,0,94]*10^(-3);
I3 = [2,0,0;0,1,0;0,0,1]*10^(-3);
I4 = eye(3,3)*10^(-3);
 
 T01 = T(:,:,1);%mudando soh os nomes
 T02 = T(:,:,2);
 T03 = T(:,:,3);
 T04 = T(:,:,4);
 
 Z0 = [0;0;1];%sempre fixo
 O0 = [0;0;0];
 ZZ = [0;0;0];

 Z1 = T01(1:3,3);%linhas 1 a 3, coluna 3
 O1 = T01(1:3,4);%linhas 1 a 3, coluna 4
 R1 = T01(1:3,1:3);
 Z2 = T02(1:3,3);%linhas 1 a 3, coluna 3
 O2 = T02(1:3,4);%linhas 1 a 3, coluna 4
 R2 = T02(1:3,1:3);
 Z3 = T03(1:3,3);%linhas 1 a 3, coluna 3
 O3 = T03(1:3,4);%linhas 1 a 3, coluna 4
 R3 = T03(1:3,1:3);
 Z4 = T04(1:3,3);%linhas 1 a 3, coluna 3
 O4 = T04(1:3,4);%linhas 1 a 3, coluna 4
 R4 = T04(1:3,1:3);
 
 
 J1 = [cross(Z0,O1-O0) ZZ ZZ ZZ;
         Z0 ZZ ZZ ZZ           ];
 J2 = [cross(Z0,O2-O0) cross(Z1,O2-O1) ZZ ZZ;
         Z0 Z1 ZZ ZZ           ];
 J3 = [cross(Z0,O3-O0) cross(Z1,O3-O1) Z2 ZZ;
         Z0 Z1 ZZ ZZ           ];
 J4 = [cross(Z0,O4-O0) cross(Z1,O4-O1) Z2 cross(Z3,O4-O3);
         Z0 Z1 ZZ Z3           ];
 
JV1 = J1(1:3,:);
JV2 = J2(1:3,:);
JV3 = J3(1:3,:);
JV4 = J4(1:3,:);
JW1 = J1(4:6,:);
JW2 = J2(4:6,:);
JW3 = J3(4:6,:);
JW4 = J4(4:6,:);
%m = [3.2,15.44,0.111, 0.1]; %kg
syms m1 m2 m3 m4 real
m = [m1;m2;m3;m4];


%dinamica
D1 = m(1) * JV1' * JV1 + JW1'*R1 * I1 * R1'*JW1;
D2 = m(2) * JV2' * JV2 + JW2'*R2 * I2 * R2'*JW2;
D3 = m(3) * JV3' * JV3 + JW3'*R3 * I3 * R3'*JW3;
D4 = m(4) * JV4' * JV4 + JW4'*R4 * I4 * R4'*JW4;
D = D1+D2+D3+D4; 
%D = simplify(D);

%% Christoffel
q = [theta1 theta2 d3 theta4];
syms theta1_v theta2_v d3_v theta4_v real
q_v = [theta1_v theta2_v d3_v theta4_v];
syms theta1_a theta2_a d3_a theta4_a real
q_a = [theta1_a theta2_a d3_a theta4_a];

for k=1:GDL
    for j=1:GDL
        for i=1:GDL
            C(k,j,i) = ( diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)) )/2;
            czin(i,j,k) =(diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)) )/2;
        end
    end
end




%% Potencial Energy

syms g
P1 = m(1)* g * O1(3);
P2 = m(2)* g * O2(3);
P3 = m(3)* g * O3(3);
P4 = m(4)* g * O4(3);
P = P1 + P2 + P3 + P4;

G_component = [diff(P,q(1)) ; diff(P,q(2)) ; diff(P,q(3)) ; diff(P,q(4))];

%MATRIZ FORM
%torques = [torque1; torque2; torque3; torque4; torque5; torque6];
C_component =  C(:,:,1) * q_v(1) + C(:,:,2) * q_v(2) + C(:,:,3) * q_v(3) + C(:,:,4) * q_v(4);
G_component = [diff(P,q(1)) ; diff(P,q(2)) ; diff(P,q(3)) ; diff(P,q(4)) ];

