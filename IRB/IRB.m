%parÃ¢metros:

a1real = 0.40 ;
a2real = 0.25;
d1real= 0.195;

%%Montagem do robo IRB
Elo(1) = Link([0 d1real a1real 0 0]);%theta, d, a, alpha, 0/1(opcional: rev/prism)
Elo(1).qlim = [-140*pi/180,140*pi/180];
Elo(2) = Link([0  0 a2real pi 0]);
Elo(1).qlim = [-150*pi/180,150*pi/180];
Elo(3) = Link([0 0 0 0 1]);
Elo(3).qlim = ([0.195-0.180, 0.195]);
Elo(4) = Link([0 0 0 0 0]);
Elo(4).qlim = [-400*pi/180,400*pi/180];
Robo2 = SerialLink(Elo);%constrói robo
Robo2.name = 'IRB';
Q0=[pi/2; pi/3; 0.4; pi/10];
Q1=[pi/7; pi/4; 0.5; pi/20];

W = [-2, 2 -2 2 -2 2];
% Movement= jtraj(Q0, Q1, (0:.1:1));
% Robo2.plot(Movement); 

syms theta1;
syms theta2;
syms theta4;
syms d3;
syms d4;
syms d1;
syms a1;
syms a2;
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



%% Calculo da matriz de transformaÃ§Ã£o:
 T(:,:,1)= A(:,:,1);
 
 for x=2:1:GDL
 
     Ttemp = A(:,:,x);
     
     T(:,:,x)= T(:,:,(x-1))*Ttemp;
      T(:,:,x)= simplify(T(:,:,x));
 
 end
 
 %Matriz de Transformacao de 1 para 2
 T(:,:, 2);

 
%  %% Area de trabalho
% 
% 
% %hold on;
% N=6000; 
% ang1=(-140*pi/180)+((280*pi/180)*rand(N,1));    %limit of joint1
% ang2=(-150*pi/180)+((300*pi/180)*rand(N,1));    %limit of joint2
% dis3=0.195-(0.195-0.180*rand(N,1));    %limit of joint3
% ang4=(-400*pi/180)+((800*pi/180)*rand(N,1));    %limit of joint4
% 
% figure(1)
% for i=1:1:N 
%     qq=[ang1(i),ang2(i),dis3(i),ang4(i)];
%     f=Robo2.fkine(qq);
%     hold on;
%     drawnow;        % This is to see the points appearing in the picture while running
%     plot3(f.t(1),f.t(2),f.t(3),'b.','MarkerSize',1.5);
% %pause(0.001) %activate it if you want to see the points that appear in
% %at each while
% end
 
 
 %% 
 
 %%calculo SINGULARIDADE
 
 %%CALCULO JACOBIANO:
 
 %%declara os vetores O e Z nescessário.

 O0=[0;0;0];
 O1=[T(1,4,1);T(2,4,1);T(3,4,1)];
 O2=[T(1,4,2);T(2,4,2);T(3,4,2)];
 O3=[T(1,4,3);T(2,4,3);T(3,4,3)];
 O4=[T(1,4,4);T(2,4,4);T(3,4,4)];
 
 Z0=[0;0;1];
 Z1=[T(1,3,1);T(2,3,1);T(3,3,1)];
 Z2=[T(1,3,2);T(2,3,2);T(3,3,2)];
 Z3=[T(1,3,3);T(2,3,3);T(3,3,3)];
 Z4=[T(1,3,4);T(2,3,4);T(3,3,4)];
 
 
 J1= [cross(Z0,O4-O0); Z0];
 J2= [cross(Z1,O4-O1); Z1];
 J3= [Z2; [0;0;0]];
 J4= [cross(Z3,O4-O3); Z3];
   
 J=[J1 J2 J3 J4]; %%%Jacobiano pronto
 
%%%SINGULARIDADE determinante matriz 4x4;

Jver=[ J(1,1) J(1,2) J(1,3) J(1,4);
       J(2,1) J(2,2) J(2,3) J(2,4);
       J(3,1) J(3,2) J(3,3) J(3,4)
       J(4,1) J(4,2) J(4,3) J(4,4)]; 

   
Jdet=[ J(1,1) J(1,2) J(1,3);
       J(2,1) J(2,2) J(2,3);
       J(3,1) J(3,2) J(3,3)];    

%%%verifica se havera singularidade
 det1= det(Jver);
 
%%%verifica quais pontos serão de singularidade
 det2=det(Jdet);
 det2=simplify(det2);
 
%% Inverse kinematics

TF = [1 0 0 0.65; 0 0 -1 -82.5; 0 1 0 -94.56; 0 0 0 1];
q = Robo2.ikine(TF,'qo',Q0,'mask',[1, 1, 1, 0, 0 ,0],'ilimit',100,'rlimit',1000,'verbose');

Movement= jtraj(Q0, q, (0:.08:1));
Robo2.plot(Movement,'workspace',W);

%% INCERTEZAS:

%valores de incerteza catalogados:
ij1= 0.01*pi/180; ij2=0.01*pi/180; ij3=0.01; ij4=0.01*pi/180;

%calculos das derivadas:
%%para posição x:
Dx=[diff(O4(1,1),theta1) diff(O4(1,1),theta2) diff(O4(1,1),d3) diff(O4(1,1),theta4)];
Dy=[diff(O4(2,1),theta1) diff(O4(2,1),theta2) diff(O4(2,1),d3) diff(O4(2,1),theta4)];
Dz=[diff(O4(3,1),theta1) diff(O4(3,1),theta2) diff(O4(3,1),d3) diff(O4(3,1),theta4)];

a1=a1real;
a2=a2real;
d1=d1real;

theta1=q(1,1);
theta2=q(1,2);
d3=q(1,3);
theta4=q(1,4);

Ex=sqrt((eval(Dx(1,1))*ij1)^2 + (eval(Dx(1,2))*ij2)^2 + (eval(Dx(1,3))*ij3)^2 +(eval(Dx(1,4))*ij4)^2);
Ey=sqrt((eval(Dy(1,1))*ij1)^2 + (eval(Dy(1,2))*ij2)^2 + (eval(Dy(1,3))*ij3)^2 +(eval(Dy(1,4))*ij4)^2);
Ez=sqrt((eval(Dz(1,1))*ij1)^2 + (eval(Dz(1,2))*ij2)^2 + (eval(Dz(1,3))*ij3)^2 +(eval(Dz(1,4))*ij4)^2);
