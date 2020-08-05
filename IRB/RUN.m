run('IRB_trajectory2.m')

omega_P_1 = 10.4841;
omega_D_1 = 18.5366;
omega_P_2 = 3.6640;
omega_D_2 = 11.5504;
omega_P_3 = 5.2280;
omega_D_3 = 25.6301;

% Control parameters for small joints (assumed to be the same)
omega_P_small = 8.7739;
omega_D_small = 38.9344;



Q_A = [0.0951    0.1937    0.0007         0];
 q_max =  2*pi*ones(1,4);        % [rad]
q_min = -2*pi*ones(1,4);        % [rad]
v_max = 3.4*ones(1,4);          % [rad/sec]
v(1,3) = 20;
% a_max = [15 15 15 25 25 25];    % [rad/sec/sec]
a_max = [50 50 50 50];    % [rad/sec/sec]
torque_max = [300 150 150 28];


 
 K_1 = [5.0000         0         0         0
          0   10.0000         0         0;
          0         0   40.0000         0;
          0         0         0   77.8688];
 K_0 = [  5         0         0         0;
          0   10         0         0;
          0         0  40.0000         0;
          0         0         0   76.9813];




