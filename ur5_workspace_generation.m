close all
clear all
clc
%% Robot construction.
deg = pi/180;
a2=425;%dimensoes (mm)
a3=392;
d1=89.2;
d2=0;
d3=0;
d4=109.3;
d5=94.75;
d6=82.5;

Elo(1) = Link([0 d1 0 pi/2]);%theta, d, a, alpha, 0/1(opcional: rev/prism)
Elo(2) = Link([0 d2 a2 0]);
Elo(3) = Link([0 d3 a3 0]);
Elo(4) = Link([0 d4 0 pi/2]);
Elo(5) = Link([0 d5 0 -pi/2]);
Elo(6) = Link([0 d6 0 0]);
Robo = SerialLink(Elo);%constrói robo
Robo.name = 'UR5';


%% Workspace Generation.
figure()
Robo.plot([0 0 0 0 0 0]);
hold on;
N=300; 
theta1=-pi+2*pi*rand(N,1);    %limit of joint1
theta2=-pi+2*pi*rand(N,1);    %limit of joint2
theta3=-pi+2*pi*rand(N,1);    %limit of joint3
theta4=-pi+2*pi*rand(N,1);    %limit of joint4
theta5=-pi+2*pi*rand(N,1);    %limit of joint5
theta6=-pi+2*pi*rand(N,1);    %limit of joint6
figure(1)   
for i=1:1:N 
    qq=[theta1(i),theta2(i),theta3(i),theta4(i),theta5(i),theta6(i)];
    f=Robo.fkine(qq);
    hold on;
    drawnow;        % This is to see the points appearing in the picture while running
    if f.t(3) > 0
        plot3(f.t(1),f.t(2),f.t(3),'b.','MarkerSize',1.5);
    end
     %pause(0.001) %activate it if you want to see the points that appear in
%at each while
end


