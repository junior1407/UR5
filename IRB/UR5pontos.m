close all
clear all
clc
%% construção

a2=0.425;%dimensoes (mm)
a3=0.392;
d1=0.0892;
d2=0;
d3=0;
d4=0.1093;
d5=0.09475;
d6=0.0825;


Elo(1) = Link([0 d1 0 pi/2]);%theta, d, a, alpha, 0/1(opcional: rev/prism)
Elo(2) = Link([0 d2 a2 0]);
Elo(3) = Link([0 d3 a3 0]);
Elo(4) = Link([0 d4 0 pi/2]);
Elo(5) = Link([0 d5 0 -pi/2]);
Elo(6) = Link([0 d6 0 0]);
Robo = SerialLink(Elo);%constrói robo
Robo.name = 'UR5';

%% matriz A(x-1 a x)
syms d1;
syms d2;
syms d3;
syms d4;
syms theta;
syms theta1;
syms theta2; 
syms theta3; 
syms theta4; 
syms theta5; 
syms theta6;

%parÃ¢metros:
%   a alpha   d theta
DH=[0   pi/2    d1   pi/2;
    0    pi/2   d2   pi/2;
    0   -pi/2     d3   -pi/2];

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



%% Calculo da matriz de transformaÃ§Ã£o:
 T(:,:,1)= A(:,:,1);
 
 for x=2:1:GDL
 
     Ttemp = A(:,:,x);
     
     T(:,:,x)= T(:,:,(x-1))*Ttemp;
     T(:,:,x)= simplify(T(:,:,x));
 
 end
 
 'Matriz de Transformacao de 1 para 2'
 T(:,:, 2)
% 1 to 3
% 1to 4 ....
%% Workspace

%hold on;
% N=3000; 
% theta1=-pi+2*pi*rand(N,1);    %limit of joint1
% theta2=-pi+2*pi*rand(N,1);    %limit of joint2
% theta3=-pi+2*pi*rand(N,1);    %limit of joint3
% theta4=-pi+2*pi*rand(N,1);    %limit of joint4
% theta5=-pi+2*pi*rand(N,1);    %limit of joint5
% theta6=-pi+2*pi*rand(N,1);    %limit of joint6
% figure(1)
% for i=1:1:N 
%     qq=[theta1(i),theta2(i),theta3(i),theta4(i),theta5(i),theta6(i)];
%     f=Robo.fkine(qq);
%     hold on;
%     drawnow;        % This is to see the points appearing in the picture while running
%     plot3(f.t(1),f.t(2),f.t(3),'b.','MarkerSize',1.5);
% %pause(0.001) %activate it if you want to see the points that appear in
% %at each while
% end

%% Inverse kinematics
T=[1 0 0 0; 0 -1 0 0; 0 0 1 -500; 0 0 0 1];
q = Robo.ikine(T, [0,0,0,0,0,0]);

%Movement= jtraj(Q0, q, (0:.08:1));
%Robo.plot(Movement);


%% Pick and place animation


% R R R X
% R R R Y
% R R R Z
% 0 0 0 1
%Resting position
start_position=[0.642; 0.088; 0.175];
default_rotation = [1 0 0 ; 0 1 0; 0 0 1];
T_start = getTransformationMatrix(start_position, default_rotation);

pick_position = [0.344; 0.473; 0.175];
T_pick = getTransformationMatrix(pick_position, default_rotation);
pick2_position = [0.344; 0.473; 0.030];
T_pick2 = getTransformationMatrix(pick2_position, default_rotation);
pick3_position = [0.344; 0.473; 0.175];
T_pick3 = getTransformationMatrix(pick3_position, default_rotation);
pick4_position = [0.344; -0.473; 0.175];
T_pick4 = getTransformationMatrix(pick4_position, default_rotation);
pick5_position = [0.344; -0.473; 0.030];
T_pick5 = getTransformationMatrix(pick5_position, default_rotation);
pick6_position = [0.344; -0.473; 0.175];
T_pick6 = getTransformationMatrix(pick6_position, default_rotation);

Q_start= Robo.ikine(T_start,'qo',[0;pi/7;0;-pi/2;-pi;-pi/2],'verbose');
Q_pick = Robo.ikine(T_pick,'q0',Q_start,'verbose');
Q_pick2 = Robo.ikine(T_pick2,'q0',Q_pick,'verbose');
Q_pick3 = Robo.ikine(T_pick3,'q0',Q_pick2,'verbose');
Q_pick4 = Robo.ikine(T_pick4,'q0',Q_pick3,'verbose');
Q_pick5 = Robo.ikine(T_pick5,'q0',Q_pick4,'verbose');
Q_pick6 = Robo.ikine(T_pick6,'q0',Q_pick5,'verbose');



Start_Pick=jtraj(Q_start, Q_pick, (0:.08:2));
Pick_Pick2 = jtraj(Q_pick, Q_pick2, (0:.08:0.8));
Pick2_Pick3 = jtraj(Q_pick2, Q_pick3, (0:.08:0.8));
Pick3_pick4 = jtraj(Q_pick3, Q_pick4,(0:.08:2) );
Pick4_pick5 = jtraj(Q_pick4, Q_pick5,(0:.08:2) );
Pick5_pick6 = jtraj(Q_pick5, Q_pick6,(0:.08:2) );

Robo.plot([Start_Pick; Pick_Pick2; Pick2_Pick3;Pick3_pick4; Pick4_pick5; Pick5_pick6]);

