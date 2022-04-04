clear all;
clc;

%% Constant Parameters + Initial Conditions
m=1;
d_h=.2;
d_v=.01;
g=9.8;
C_TQ=.1;
k_x=1;
k_v=1.4;
k_R=1.7;
k_omega=2.4;
J=10^-3*diag([1,1,5]);
x0=[0, 0, 0]';
v0=[0, 0, 0]';
R0=eye(3);
omega0=[0, 0, 0]';

P= [m;d_h;d_v;g;C_TQ;k_x;k_v;k_R;k_omega;J(:,1);J(:,2);J(:,3);x0;v0;R0(:,1);R0(:,2);R0(:,3);omega0];

