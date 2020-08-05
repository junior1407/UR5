close all
clear all
clc
addpath('functions')
%%Robot data
deg = pi/180;


syms a2 a3 d1 d4 d5 d6
%a2=425/1000;%dimensoes (mm)
%a3=392/1000;
%d1=89.2/1000;
d2=0;
d3=0;
%d4=109.3/1000;
%d5=94.75/1000;
%d6=82.5/1000;

%% matriz A(x-1 a x)

syms x;
syms theta1 real;
syms theta2 real; 
syms theta3 real; 
syms theta4 real; 
syms theta5 real; 
syms theta6 real;

q = [theta1 theta2 theta3 theta4 theta5 theta6];
syms theta1_v theta2_v theta3_v theta4_v theta5_v theta6_v
q_v = [theta1_v theta2_v theta3_v theta4_v theta5_v theta6_v];
syms theta1_a theta2_a theta3_a theta4_a theta5_a theta6_a
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

J6 = [cross(Z0,O6-O0) cross(Z1,O6-O1) cross(Z0,O6-O2) cross(Z1,O6-O3) cross(Z0,O5-O4) cross(Z1,O6-O5);
        Z0 Z1 Z2 Z3 Z4 Z5];
%Dividing the Jacobian in segments
J11 = J6(1:3,1:3);
J22 = J6(4:6,4:6);

%Arm singularities are given by det(J11) = 0
%dJ11 = simplify(det(J11), 'IgnoreAnalyticConstraints',true);
%Sol = solve(dJ11 == 0, 'ReturnConditions',true, 'Real',true);

%Wrist Singularities are given by det(J22) = 0 
%dJ22 =simplify(det(J22));
%Sol2 = solve(dJ22 ==0);
