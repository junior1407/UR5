function [ M, G, C_dq ] = IRB_getMatrices( q, dq )
%Returns the dynamics of the UR5 manipulator

a1 = 0.40;
a2 = 0.25;
d1= 0.195;
d4=0;

m1 = 3.2;
m2 = 15.44;
m3 =0.1;
m4 = 0.1;

g = 9.8;
theta1 = q(1);
theta2 = q(2);
d3 = q(3);
theta4 = q(4);

theta1_v = dq(1);
theta2_v = dq(2);
d3_v = dq(3);
theta4_v = dq(4);

M = [ (18851*a1^2)/1000 + (15651*cos(theta2)*a1*a2)/500 + (15651*a2^2)/1000 + 1/10, (15651*a2^2)/1000 + (15651*a1*cos(theta2)*a2)/1000 + 12/125,        0, -1/1000;
                  (15651*a2^2)/1000 + (15651*a1*cos(theta2)*a2)/1000 + 12/125,                                  (15651*a2^2)/1000 + 12/125,        0, -1/1000;
                                                                            0,                                                           0, 211/1000,       0;
                                                                      -1/1000,                                                     -1/1000,        0,  1/1000
    ];
C = [ -a1*a2*theta2_v*sin(theta2)*(m2 + m3 + m4), -a1*a2*sin(theta2)*(theta1_v + theta2_v)*(m2 + m3 + m4), 0, 0;
  a1*a2*theta1_v*sin(theta2)*(m2 + m3 + m4),                                                       0, 0, 0;
                                          0,                                                       0, 0, 0;
                                          0,                                                       0, 0, 0];
G=[0;0;-(211*g)/1000;0];
C_dq = C*dq;


