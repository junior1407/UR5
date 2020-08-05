%parâmetros:
clc;
close all;
clear all;


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
 
 O4 = T04(1:3,4);%linhas 1 a 3, coluna 4
 X =O4(1);
 Y = O4(2);
 Z = O4(3);
utheta1= 0.01*pi/180;
utheta2= 0.01*pi/180;
ud3=0.01;
utheta4=0.01*pi/180;
a1 = 0.40;
a2 = 0.25;
d1= 0.195;
d4=0;

UX = ( (diff(X,theta1) * utheta1 )^2 +  (diff(X,theta2) * utheta2 )^2 + (diff(X,d3) * ud3 )^2  )^(1/2);
UY = ( (diff(Y,theta1) * utheta1 )^2 +  (diff(Y,theta2) * utheta2 )^2 + (diff(Y,d3) * ud3 )^2  )^(1/2);
UZ = ( (diff(Z,theta1) * utheta1 )^2 +  (diff(Z,theta2) * utheta2 )^2 + (diff(Z,d3) * ud3 )^2  )^(1/2);


A = [0.0743    0.1612    0.0200         0];
B = [0.5923    0.9300    0.0200         0];
C = [0.5923    0.9300    0.1650         0];
D = [0.5923    0.9300    0.0200         0];
E = [-0.5923   -0.9300    0.0200         0];
F = [-0.5923   -0.9300    0.1650         0];

U_A = [eval(subs(UX, [theta1 theta2 d3 theta4], A)) eval(subs(UY, [theta1 theta2 d3 theta4], A)) eval(subs(UZ, [theta1 theta2 d3 theta4], A))]
U_B = [eval(subs(UX, [theta1 theta2 d3 theta4], B)) eval(subs(UY, [theta1 theta2 d3 theta4], B)) eval(subs(UZ, [theta1 theta2 d3 theta4], B))]
U_C = [eval(subs(UX, [theta1 theta2 d3 theta4], C)) eval(subs(UY, [theta1 theta2 d3 theta4], C)) eval(subs(UZ, [theta1 theta2 d3 theta4], C))]
U_D = [eval(subs(UX, [theta1 theta2 d3 theta4], D)) eval(subs(UY, [theta1 theta2 d3 theta4], D)) eval(subs(UZ, [theta1 theta2 d3 theta4], D))]
U_E = [eval(subs(UX, [theta1 theta2 d3 theta4], E)) eval(subs(UY, [theta1 theta2 d3 theta4], E)) eval(subs(UZ, [theta1 theta2 d3 theta4], E))]
U_F = [eval(subs(UX, [theta1 theta2 d3 theta4], F)) eval(subs(UY, [theta1 theta2 d3 theta4], F)) eval(subs(UZ, [theta1 theta2 d3 theta4], F))]
