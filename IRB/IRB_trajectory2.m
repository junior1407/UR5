%clc
%clear
%close all
addpath('functions')
%% Robot construction.

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
Robo = SerialLink(Elo);%constrói robo
%% Getting KeyPoints


A =[0.642; 0.088; 0.175];
B=[0.344; 0.473; 0.175];
C=[0.344; 0.473; 0.030];
D=[0.344; 0.473; 0.175];
E=[0.344; -0.473; 0.175];
F=[0.344; -0.473; 0.030];

default_rotation = [1 0 0 ; 0 0 -1; 0  1 0];
T_A = getTransformationMatrix(A, default_rotation);
Q_A= Robo.ikine(T_A,'mask',[1, 1, 1, 0, 0 ,0], 'verbose');

T_B = getTransformationMatrix(B, default_rotation);
Q_B= Robo.ikine(T_B,'mask',[1, 1, 1, 0, 0 ,0], 'verbose');

T_C = getTransformationMatrix(C, default_rotation);
Q_C= Robo.ikine(T_C,'mask',[1, 1, 1, 0, 0 ,0], 'verbose');

T_D = getTransformationMatrix(D, default_rotation);
Q_D= Robo.ikine(T_D,'mask',[1, 1, 1, 0, 0 ,0], 'verbose');

T_E = getTransformationMatrix(E, default_rotation);
Q_E= Robo.ikine(T_E,'mask',[1, 1, 1, 0, 0 ,0], 'verbose');

T_F = getTransformationMatrix(F, default_rotation);
Q_F= Robo.ikine(T_F,'mask',[1, 1, 1, 0, 0 ,0], 'verbose');

traj1=jtraj(Q_A, Q_B, (0:.08:2));
traj2=jtraj(Q_B, Q_C, (0:.08:2));
traj3=jtraj(Q_C, Q_D, (0:.08:2));
traj4=jtraj(Q_D, Q_E, (0:.08:2));
traj5=jtraj(Q_E, Q_F, (0:.08:2));
%Robo.plot([traj1; traj2; traj3; traj4; traj5]);

%Q_pick = Robo.ikine(T_pick, 'verbose');
%Q_pick2 = Robo.ikine(T_pick2, 'verbose');   
% 
% %% Interpolating
% %Theta0
 waypoints = [Q_A; Q_B; Q_C; Q_D; Q_E; Q_F];
 
theta_space = zeros(4,6);
for i=1:4
    for j = 1:6
        theta_space(i,j) = [waypoints(j,i)]; 
    end
end
dt = 0.008;
 
[x1_s, th1_s] = computePositionIRB(theta_space(1,:), dt,0);
[x1_v, y1_v, x1_a, y1_a] = calcVelocityAcceleration(x1_s, th1_s);
signal1 = [x1_s' th1_s'];
signal1_v = [x1_v' y1_v'];
signal1_a = [x1_a' y1_a'];

[x2_s, th2_s] = computePositionIRB(theta_space(2,:), dt,0);
[x2_v, y2_v, x2_a, y2_a] = calcVelocityAcceleration(x2_s, th2_s);
signal2 = [x2_s' th2_s'];
signal2_v = [x2_v' y2_v'];
signal2_a = [x2_a' y2_a'];

[x3_s, th3_s] = computePositionIRB(theta_space(3,:), dt,'poly3');
th3_s = abs(th3_s)
%th3_s = abs(th3_s)
[x3_v, y3_v, x3_a, y3_a] = calcVelocityAcceleration(x3_s, th3_s);
signal3 = [x3_s' th3_s'];
signal3_v = [x3_v' y3_v'];
signal3_a = [x3_a' y3_a'];

[x4_s, th4_s] = computePositionIRB(theta_space(4,:), dt,0);
[x4_v, y4_v, x4_a, y4_a] = calcVelocityAcceleration(x4_s, th4_s);
signal4 = [x4_s' th4_s'];
signal4_v = [x4_v' y4_v'];
signal4_a = [x4_a' y4_a'];




% 
% %% Plotting
% figure()
% subplot(3,1,1)
% plot(x1_s, th1_s )
% xlabel('t')
% ylabel('Positions')
% subplot(3,1,2)
% plot(x1_v, y1_v)
% xlabel('t')
% ylabel('Velocities')
% subplot(3,1,3)
% plot(x1_a, y1_a)
% xlabel('t')
% ylabel('Acceleration')
% 
% 
% figure()
% 
% for i=1:200:length(th1_s)
%     Tm = Robo.fkine([th1_s(i), th2_s(i), th3_s(i), th4_s(i)]);
%     x(i) = Tm.t(1);
%     y(i) = Tm.t(2);
%     z(i) = Tm.t(3);
%     Robo.plot([th1_s(i), th2_s(i), th3_s(i), th4_s(i)]);
%     drawnow;
%     hold on;
%     plot3(x(i),y(i),z(i),'b.');
%     %Robo.plot([th1_s(i), th2_s(i), th3_s(i), th4_s(i), th5_s(i), th6_s(i)])
% end
