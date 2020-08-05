close all
clear all
clc

%%Robot data
deg = pi/180;
a2=425;%dimensoes (mm)
a3=392;
d1=89.2;
d2=0;
d3=0;
d4=109.3;
d5=94.75;
d6=82.5;

%% matriz A(x-1 a x)

syms x;
syms theta;
syms theta1;
syms theta2; 
syms theta3; 
syms theta4; 
syms theta5; 
syms theta6;

%parâmetros:
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

'Matriz do Link 1'
A(:,:, 1)

'Matriz do Link 2'
A(:,:, 2)


'Matriz do Link 3'
A(:,:, 3)

'Matriz do Link 4'
A(:,:, 4)


'Matriz do Link 5'
A(:,:, 5)


'Matriz do Link 6'
A(:,:, 6)



%% Calculo da matriz de transformação:
 T(:,:,1)= A(:,:,1);
 
 for x=2:1:GDL
 
     Ttemp = A(:,:,x);
     
     T(:,:,x)= T(:,:,(x-1))*Ttemp;
     T(:,:,x)= simplify(T(:,:,x));
 
 end
 
'Matriz de Transformacao de 1 para 2'
T(:,:, 2)  
 
'Matriz de Transformacao de 1 para 3'
T(:,:, 3)  
 
'Matriz de Transformacao de 1 para 4'
T(:,:, 4)  

'Matriz de Transformacao de 1 para 5'
T(:,:, 5)

'Matriz de Transformacao de 1 para 6'
T(:,:, 6)  


