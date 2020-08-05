%close all
%clear all
%clc
addpath('functions')
%%Robot data
deg = pi/180;
a2=425/1000;%dimensoes (mm)
a3=392/1000;
d1=89.2/1000;
d2=0;
d3=0;
d4=109.3/1000;
d5=94.75/1000;
d6=82.5/1000;

%% matriz A(x-1 a x)

syms x;
syms theta1 real;
syms theta2 real; 
syms theta3 real; 
syms theta4 real; 
syms theta5 real; 
syms theta6 real;

q = [theta1 theta2 theta3 theta4 theta5 theta6];
syms theta1_v theta2_v theta3_v theta4_v theta5_v theta6_v real
q_v = [theta1_v theta2_v theta3_v theta4_v theta5_v theta6_v];
syms theta1_a theta2_a theta3_a theta4_a theta5_a theta6_a real
q_a = [theta1_a theta2_a theta3_a theta4_a theta5_a theta6_a];
%parÃ¢metros:
%   a alpha   d theta
DH=[0   pi/2    d1   theta1;
    a2    0   d2   theta2;
    a3   0     d3   theta3;
    0   pi/2     d4   theta4;
    0   -pi/2   d5   theta5;
    0    0    d6   theta6];

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

%% Calculo da matriz de transformação:
 T(:,:,1)= A(:,:,1);
 
 for x=2:1:GDL
 
     Ttemp = A(:,:,x);
     
     T(:,:,x)= T(:,:,(x-1))*Ttemp;
     T(:,:,x)= simplify(T(:,:,x));
 
 end
% 
%  'Matriz A1'
%  T(:,:,1)
%  
%  'Matriz A2'
%  T(:,:,2)
 
 
 
 T01 = T(:,:,1);%mudando soh os nomes
 T02 = T(:,:,2);
 T03 = T(:,:,3);
 T04 = T(:,:,4);
 T05 = T(:,:,5);
 T06 = T(:,:,6);
 
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
 Z5 = T05(1:3,3);%linhas 1 a 3, coluna 3
 O5 = T05(1:3,4);%linhas 1 a 3, coluna 4
 R5 = T05(1:3,1:3);
 Z6 = T06(1:3,3);%linhas 1 a 3, coluna 3
 O6 = T06(1:3,4);%linhas 1 a 3, coluna 4
 R6 = T06(1:3,1:3); 
 
J1 = [cross(Z0,O1-O0) ZZ ZZ ZZ ZZ ZZ;
        Z0 ZZ ZZ ZZ ZZ ZZ];
    
J2 = [cross(Z0,O2-O0) cross(Z1,O2-O1) ZZ ZZ ZZ ZZ;
        Z0 Z1 ZZ ZZ ZZ ZZ];

J3 = [cross(Z0,O3-O0) cross(Z0,O3-O1) cross(Z1,O3-O2) ZZ ZZ ZZ;
        Z0 Z1 Z2 ZZ ZZ ZZ];
J4 = [cross(Z0,O4-O0) cross(Z1,O4-O1) cross(Z0,O4-O2) cross(Z1,O4-O3) ZZ ZZ;
        Z0 Z1 Z2 Z3 ZZ ZZ];
J5 = [cross(Z0,O5-O0) cross(Z1,O5-O1) cross(Z0,O5-O2) cross(Z0,O5-O3) cross(Z1,O5-O4) ZZ;
        Z0 Z1 Z2 Z3 Z4 ZZ];
J6 = [cross(Z0,O6-O0) cross(Z1,O6-O1) cross(Z0,O6-O2) cross(Z1,O6-O3) cross(Z0,O5-O4) cross(Z1,O6-O5);
        Z0 Z1 Z2 Z3 Z4 Z5];
 
I1 = [84,0,0;0,64,0;0,0,84]*10^(-4);
I2 = [78,0,0;0,21,0;0,0,21]*10^(-4);
I3 = [16,0,0;0,462,0;0,0,462]*10^(-4);
I4 = [16,0,0;0,16,0;0,0,9]*10^(-4);
I5 = [16,0,0;0,16,0;0,0,9]*10^(-4);
I6 = eye(3)*10^(-4);
 

JV1 = J1(1:3,:);
JV2 = J2(1:3,:);
JV3 = J3(1:3,:);
JV4 = J4(1:3,:);
JV5 = J5(1:3,:);
JV6 = J6(1:3,:);
JW1 = J1(4:6,:);
JW2 = J2(4:6,:);
JW3 = J3(4:6,:);
JW4 = J4(4:6,:);
JW5 = J5(4:6,:);
JW6 = J6(4:6,:);
m = [3.7,8.393,2.275,1.219,1.219,.1879]; %kg

%dinamica
D1 = m(1) * JV1' * JV1 + JW1'*R1 * I1 * R1'*JW1;
D2 = m(2) * JV2' * JV2 + JW2'*R2 * I2 * R2'*JW2;
D3 = m(3) * JV3' * JV3 + JW3'*R3 * I3 * R3'*JW3;
D4 = m(4) * JV4' * JV4 + JW4'*R4 * I4 * R4'*JW4;
D5 = m(5) * JV5' * JV5 + JW5'*R5 * I5 * R5'*JW5;
D6 = m(6) * JV6' * JV6 + JW6'*R6 * I6 * R6'*JW6;

D = D1+D2+D3+D4+D5+D6;
%D = simplify(D);

%% Christoffel


for k=1:GDL
    for j=1:GDL
        for i=1:GDL
            c(i,j,k)=( diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)) )/2;
            C(j,k,i) = ( diff(D(k,j),q(i)) + diff(D(k,i),q(j)) - diff(D(i,j),q(k)) )/2;
        end
    end
end



%% Potencial Energy

syms g
P1 = m(1)* g * O1(3);
P2 = m(2)* g * O2(3);
P3 = m(3)* g * O3(3);
P4 = m(4)* g * O4(3);
P5 = m(5)* g * O5(3);
P6 = m(6)* g * O6(3);
P = P1 + P2 + P3 + P4 + P5 + P6;

G_component = [diff(P,q(1)) ; diff(P,q(2)) ; diff(P,q(3)) ; diff(P,q(4)) ; diff(P,q(5)) ; diff(P,q(6))];

    
%Torques
%Torque k =  Dkj * q`` + Cijk * q`* q`` + diff(P, qk),  there are 6 k`s. i
%and j range from 1 to n.

torque = sym(zeros(1,GDL));
for k=1:GDL
    torque(k) = torque(k) + diff(P,q(k)); % Adding gravity Component
    for j=1:GDL
        torque(k) = torque(k) + D(k,j)*q_a(k); % Adding D component
        for i=1:GDL
            torque(k) = torque(k) + c(i,j,k)*q_v(i)*q_v(j); % Adding C component
        end 
    end
end
syms torque1 torque2 torque3 torque4 torque5 torque6 real
%solTor1 = solve(torque(1) == torque1, theta1_a)

%MATRIZ FORM
%torques = [torque1; torque2; torque3; torque4; torque5; torque6];
C_component =  C(:,:,1) * q_v(1) + C(:,:,2) * q_v(2) + C(:,:,3) * q_v(3) + C(:,:,4) * q_v(4) + C(:,:,5) * q_v(5) + C(:,:,6) * q_v(6);
%G_component = [diff(P,q(1)) ; diff(P,q(2)) ; diff(P,q(3)) ; diff(P,q(4)) ; diff(P,q(5)) ; diff(P,q(6))];




%torques = D * q_a' + C_component * q_v' + G_component;
%torquess = [torque1; torque2; torque3; torque4; torque5; torque6];
%accelerations = inv(D) * (torquess - C_component * q_v' - G_component);
%   

