close all
clear all
clc
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
 
 
T06 = T(:,:,6);
O6 = T06(1:3,4);%linhas 1 a 3, coluna 4
X =O6(1);
Y = O6(2);
Z = O6(3);
utheta=0.01*pi/180;
UX=0;
UY=0;
UZ=0;
for i=1:6
   UX = UX + (diff(X,q(i)) * utheta)^2; 
   UY = UY + (diff(Y,q(i)) * utheta)^2; 
   UZ = UZ + (diff(Z,q(i)) * utheta)^2; 
end
UX = sqrt(UX);
UY = sqrt(UY);
UZ = sqrt(UZ);



U_A = [eval(subs(UX, q, Q_A)) eval(subs(UY, q, Q_A)) eval(subs(UZ, q, Q_A))];
U_B = [eval(subs(UX, q, Q_B)) eval(subs(UY, q, Q_B)) eval(subs(UZ, q, Q_B))];
U_C = [eval(subs(UX, q, Q_C)) eval(subs(UY, q, Q_C)) eval(subs(UZ, q, Q_C))];
U_D = [eval(subs(UX, q, Q_D)) eval(subs(UY, q, Q_D)) eval(subs(UZ, q, Q_D))];
U_E = [eval(subs(UX, q, Q_E)) eval(subs(UY, q, Q_E)) eval(subs(UZ, q, Q_E))];
U_F = [eval(subs(UX, q, Q_F)) eval(subs(UY, q, Q_F)) eval(subs(UZ, q, Q_F))];
 