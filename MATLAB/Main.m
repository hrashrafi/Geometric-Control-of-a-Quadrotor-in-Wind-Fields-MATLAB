clear all;
clc;

%% Initial Conditions
x0=[0, 0, 0]'; % Initial Position
v0=[0, 0, 0]';  % Initial Velocity
R0=eye(3);   % Initial Rotation Matrix
omega0=[0, 0, 0]';  % Initial Angular Velocity

%% Numerical Solution
[t,s]=ode45(@Quadrotor,[0 40],[x0;v0;R0(:,1);R0(:,2);R0(:,3);omega0]); % Duration: 0 to 40S

%% Error Computation
e_x=Gen_e_x(t,s);  % Position Error
e_R=Gen_e_R(t,s);  % Attitude Error

%% Ploting Errors


subplot(3,2,1)
plot(t,e_x(:,1))
ylabel('e_x_1')
grid on

subplot(3,2,2)
plot(t,e_R(:,1))
ylabel('e_R_1')
grid on

subplot(3,2,3)
plot(t,e_x(:,2))
ylabel('e_x_2')
axis([0 40 -.1 .1])
grid on

subplot(3,2,4)
plot(t,e_R(:,2))
ylabel('e_R_2')
grid on

subplot(3,2,5)
plot(t,e_x(:,3))
xlabel('Time (s)')
ylabel('e_x_3')
grid on

subplot(3,2,6)
plot(t,e_R(:,3))
xlabel('Time (s)')
ylabel('e_R_3')
axis([0 40 -.002 .002])
grid on