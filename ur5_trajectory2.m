%clc
%clear
%close all
addpath('functions')
%% Robot construction.
deg = pi/180;
a2=0.425;%dimensoes (m)
a3=0.39243;
d1 = 0.0892;
d2=0;
d3=0;
d4=0.109;
d5=0.093;
d6=0.082;

Elo(1) = Link([0 d1 0 pi/2]);%theta, d, a, alpha, 0/1(opcional: rev/prism)
Elo(2) = Link([0 d2 a2 0]);
Elo(3) = Link([0 d3 a3 0]);
Elo(4) = Link([0 d4 0 pi/2]);
Elo(5) = Link([0 d5 0 -pi/2]);
Elo(6) = Link([0 d6 0 0]);
Robo = SerialLink(Elo);%constrói robo
Robo.name = 'UR5';

%% Getting KeyPoints


A =[0.642; 0.088; 0.175];
B=[0.344; 0.473; 0.175];
C=[0.344; 0.473; 0.030];
D=[0.344; 0.473; 0.175];
E=[0.344; -0.473; 0.175];
F=[0.344; -0.473; 0.030];

default_rotation = [1 0 0 ; 0 0 -1; 0  1 0];
T_A = getTransformationMatrix(A, default_rotation);
Q_A= Robo.ikine(T_A, 'verbose');

T_B = getTransformationMatrix(B, default_rotation);
Q_B= Robo.ikine(T_B, 'verbose');

T_C = getTransformationMatrix(C, default_rotation);
Q_C= Robo.ikine(T_C, 'verbose');

T_D = getTransformationMatrix(D, default_rotation);
Q_D= Robo.ikine(T_D, 'verbose');

T_E = getTransformationMatrix(E, default_rotation);
Q_E= Robo.ikine(T_E, 'verbose');

T_F = getTransformationMatrix(F, default_rotation);
Q_F= Robo.ikine(T_F, 'verbose');

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
 
theta_space = zeros(6,5);
for i=1:6
    for j = 1:6
        theta_space(i,j) = [waypoints(j,i)]; 
    end
end
dt = 0.008;
 
[x1_s, th1_s] = computePosition(theta_space(1,:), dt);
[x1_v, y1_v, x1_a, y1_a] = calcVelocityAcceleration(x1_s, th1_s);
signal1 = [x1_s' th1_s'];
signal1_v = [x1_v' y1_v'];
signal1_a = [x1_a' y1_a'];

[x2_s, th2_s] = computePosition(theta_space(2,:), dt);
[x2_v, y2_v, x2_a, y2_a] = calcVelocityAcceleration(x2_s, th2_s);
signal2 = [x2_s' th2_s'];
signal2_v = [x2_v' y2_v'];
signal2_a = [x2_a' y2_a'];

[x3_s, th3_s] = computePosition(theta_space(3,:), dt);
[x3_v, y3_v, x3_a, y3_a] = calcVelocityAcceleration(x3_s, th3_s);
signal3 = [x3_s' th3_s'];
signal3_v = [x3_v' y3_v'];
signal3_a = [x3_a' y3_a'];

[x4_s, th4_s] = computePosition(theta_space(4,:), dt);
[x4_v, y4_v, x4_a, y4_a] = calcVelocityAcceleration(x4_s, th4_s);
signal4 = [x4_s' th4_s'];
signal4_v = [x4_v' y4_v'];
signal4_a = [x4_a' y4_a'];

[x5_s, th5_s] = computePosition(theta_space(5,:), dt);
%th5_s = 0.4*ones(size(th5_s));
[x5_v, y5_v, x5_a, y5_a] = calcVelocityAcceleration(x5_s, th5_s);
signal5 = [x5_s' th5_s'];
signal5_v = [x5_v' y5_v'];
signal5_a = [x5_a' y5_a'];

[x6_s, th6_s] = computePosition(theta_space(6,:), dt);
%th6_s = 0.4*ones(size(th6_s));
[x6_v, y6_v, x6_a, y6_a] = calcVelocityAcceleration(x6_s, th6_s);
signal6 = [x6_s' th6_s'];
signal6_v = [x6_v' y6_v'];
signal6_a = [x6_a' y6_a'];




%% Plotting
%  
% subplot(3,1,1)
% plot(x1_s, th2_s )
% xlabel('t')
% ylabel('Positions')
% subplot(3,1,2)
% plot(x2_v, y2_v)
% xlabel('t')
% ylabel('Velocities')
% subplot(3,1,3)
% plot(x2_a, y2_a)
% xlabel('t')
% ylabel('Acceleration')
% 
% 
% 
%

figure()
for i=1:30:length(th1_s)
    Tm = Robo.fkine([th1_s(i), th2_s(i), th3_s(i), th4_s(i), th5_s(i), th6_s(i)]);
    x = Tm.t(1);
    y = Tm.t(2);
    z = Tm.t(3);
    plot3(x,y,z,'b.');
    drawnow;
    hold on;
   % Robo.plot([th1_s(i), th2_s(i), th3_s(i), th4_s(i), th5_s(i), th6_s(i)])
end


for i=1:50:5419
    T1 = Robo.fkine([out.posicaoReal.Data(i,1) out.posicaoReal.Data(i,2) out.posicaoReal.Data(i,3) out.posicaoReal.Data(i,4) out.posicaoReal.Data(i,5) out.posicaoReal.Data(i,6)]);
    x = T1.t(1);
    y = T1.t(2);
    z = T1.t(3);
    plot3(x,y,z,'r.');
    drawnow;
    hold on;
    %Robo.plot([th1_s(i), th2_s(i), th3_s(i), th4_s(i), th5_s(i), th6_s(i)])
end
hold off

