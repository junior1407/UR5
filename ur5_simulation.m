close all
clear all
clc
addpath('functions')
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

%% Pick and place animation

% R R R X
% R R R Y
% R R R Z
% 0 0 0 1

default_rotation = [1 0 0 ; 0 0 -1; 0  1 0];

start_position=[817; -191.8; 100];
pick_position = [191.8; 504.6; 100];
pick2_position = [191.8; 504.6; 0];

T_start = getTransformationMatrix(start_position, default_rotation);
T_pick = getTransformationMatrix(pick_position, default_rotation);
T_pick2 = getTransformationMatrix(pick2_position, default_rotation);

Q_start= Robo.ikine(T_start, 'verbose');
Q_pick = Robo.ikine(T_pick, 'verbose');
Q_pick2 = Robo.ikine(T_pick2, 'verbose');

Start_Pick=jtraj(Q_start, Q_pick, (0:.08:2));
Pick_Pick2 = jtraj(Q_pick, Q_pick2, (0:.08:0.8));
Pick2_Pick = jtraj(Q_pick2, Q_pick, (0:.08:0.8));
Pick_Start = jtraj(Q_pick, Q_start,(0:.08:2) );
Robo.plot([Start_Pick; Pick_Pick2; Pick2_Pick; Pick_Start]);
