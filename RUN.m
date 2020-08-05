run('ur5_trajectory2.m')

omega_P_1 = 19.4841;
omega_D_1 = 36.5366;
omega_P_2 = 6.6640;
omega_D_2 = 22.5504;
omega_P_3 = 10.2280;
omega_D_3 = 56.6301;

% Control parameters for small joints (assumed to be the same)
omega_P_small = 8.7739;
omega_D_small = 38.9344;


K0_11 = [omega_P_1^2, 0, 0; 0, omega_P_2^2, 0; 0, 0, omega_P_3^2];    
K_0 = [K0_11, zeros(3,3); zeros(3,3),omega_P_small^2*eye(3,3)];
K1_11 = [omega_D_1*2, 0, 0; 0, omega_D_2*2, 0; 0, 0, omega_D_3*2];
K_1 = [K1_11, zeros(3,3); zeros(3,3),2*omega_D_small*eye(3,3)];
     
 q_max =  2*pi*ones(1,6);        % [rad]
q_min = -2*pi*ones(1,6);        % [rad]
v_max = 3.4*ones(1,6);          % [rad/sec]
% a_max = [15 15 15 25 25 25];    % [rad/sec/sec]
a_max = [27 27 27 27 27 27];    % [rad/sec/sec]
torque_max = [150 150 150 28 28 28];