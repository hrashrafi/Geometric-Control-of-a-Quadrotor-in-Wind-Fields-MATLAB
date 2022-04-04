function y = Quadrotor(t,s)
%% Current States
x=s(1:3);
v=s(4:6);
R=[s(7:9),s(10:12),s(13:15)];
omega=s(16:18);

%% Constant Parameters
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
r1=[d_h, 0, d_v]';
r2=[0, d_h, d_v]';
r3=[-d_h,0,d_v]';
r4=[0,-d_h,d_v]';
C_d=0.01;
%% Desired Trajectory

x_d=[cos(2*t), 0, 0]';
xdot_d=[-2*sin(2*t), 0, 0]';
xdot2_d=[-4*cos(2*t), 0, 0]';
xdot3_d=[8*sin(2*t), 0, 0]';
xdot4_d=[16*cos(2*t), 0, 0]';

b_1d=[1,0,0]';
bdot_1d=zeros(3,1);
bdot2_1d=zeros(3,1);

%% Disturbance Force
v_w=[sin(2*t)+cos(4*t),cos(3*t),-.5]';
F_d=-C_d*norm(v-v_w)*(v-v_w);

%% omega_hat
omega_hat= [0, -omega(3), omega(2)
             omega(3), 0, -omega(1)
               -omega(2), omega(1), 0];

Rdot= R*omega_hat;

%% Position Error 

e_x=x-x_d;
e_v=v-xdot_d;

%% A & f & f1

A= -k_x*e_x-k_v*e_v-m*g*[0, 0, 1]'+m*xdot2_d;
f1=-A;
f= -A'*R*[0,0,1]';

%% Desired Rotation Matrix & Angular Velocity + Derivatives

% First and second derivative of e_v:
edot_v=g*[0,0,1]'- f* R*[0,0,1]'/m- xdot2_d;
fdot=(k_x*e_v+k_v*edot_v-m* xdot3_d)'*R(:,3)+f1'*Rdot(:,3);
edot2_v=(-fdot*R(:,3)-f*Rdot(:,3)-m*xdot3_d)/m;

% b_3 vector:
Adot=-k_x*e_v-k_v*edot_v+m*xdot3_d;
Adot2=-k_x*edot_v-k_v*edot2_v+m*xdot4_d;
b_3c=-A/norm(A);

% b_2 vector:
C=cross(b_1d,b_3c);
n_C=norm(C);
b_2c=-C/n_C;

% b_1 vector:
b_1c=cross(b_2c,b_3c);

% Desired R:
R_c=[b_1c,b_2c,b_3c];

% Derivative of b_3:
n_A=norm(A);
bdot_3c=-(Adot/n_A)+(A*(Adot'*A))/(n_A^3);

% Derivative of b_2:
Cdot=cross(b_1d,bdot_3c)+cross(bdot_1d,b_3c);
bdot_2c=-(Cdot/n_C)+(C*(Cdot'*C))/(n_C^3);

% Derivative of b_1:
bdot_1c=cross(bdot_2c,b_3c)+cross(b_2c,bdot_3c);

% Derivative of desired R:
Rdot_c=[bdot_1c,bdot_2c,bdot_3c];

% Second derivative of b_3:
E0=-Adot2/n_A;
E1=2*(A'*Adot)*Adot/(n_A^3);
E2=(norm(Adot)^2+A'*Adot2)*A/(n_A^3);
E3=-3*(A'*Adot)^2*A/(n_A^5);
bdot2_3c=E0+E1+E2+E3;

% Second derivative of b_2:
Cdot2=cross(b_1d,bdot2_3c)+2*cross(bdot_1d,bdot_3c)+cross(bdot2_1d,b_3c);
E0=-Cdot2/n_C;
E1=2*(C'*Cdot)*Cdot/(n_C^3);
E2=(norm(Cdot)^2+C'*Cdot2)*C/(n_C^3);
E3=-3*(C'*Cdot)^2*C/(n_C^5);
bdot2_2c=E0+E1+E2+E3;

% Second derivative of b_1:
bdot2_1c=cross(bdot2_2c,b_3c)+2*cross(bdot_2c,bdot_3c)+cross(b_2c,bdot2_3c);

% Second derivative of commanded R:
Rdot2_c=[bdot2_1c,bdot2_2c,bdot2_3c];

% omega_c:
omega_c_before_vee=R_c'*Rdot_c;
omega_c= [omega_c_before_vee(3,2), omega_c_before_vee(1,3), omega_c_before_vee(2,1)]';

% omegadot_c:
omega_c_hat=omega_c_before_vee;
omegadot_c_before_vee= R_c'*Rdot2_c-omega_c_hat^2;
omegadot_c= [omegadot_c_before_vee(3,2), omegadot_c_before_vee(1,3), omegadot_c_before_vee(2,1)]';

%% Rotational Error 

e_R_before_vee= .5*(R_c'*R-R'*R_c);
e_R= [e_R_before_vee(3,2), e_R_before_vee(1,3), e_R_before_vee(2,1)]';
e_omega= omega-R'*R_c*omega_c;

%% Controller

%f= -A'*R*[0,0,1]'; computed
M= -k_R*e_R-k_omega*e_omega+cross(omega,J*omega)-J*(omega_hat*R'*R_c*omega_c-R'*R_c*omegadot_c);

%% Rotor Controller!!
Tmat= [1, 0 , 2/d_h, -1/C_TQ
           1, -2/d_h, 0, 1/C_TQ
           1, 0, -2/d_h, -1/C_TQ
           1, 2/d_h, 0, 1/C_TQ];
T=.25*Tmat*[f; M(1); M(2); M(3)];

%% System Dynamics

U_e= m*g*[0,0,1]'-(T(1)+T(2)+T(3)+T(4))*R*[0,0,1]'+F_d;
M_e= -((cross(r1,T(1)*[0,0,1]')+cross(r2,T(2)*[0,0,1]')...
      +cross(r3,T(3)*[0,0,1]')+cross(r4,T(4)*[0,0,1]')))-C_TQ*(T(1)-T(2)+T(3)-T(4))*[0,0,1]';
%U_e= m*g*[0,0,1]'-(f)*R*[0,0,1]';
%M_e=M;
vdot= U_e/m;
Rdot= R*omega_hat;
omegadot= J\(M_e-cross(omega,J*omega));
y=[v;vdot;Rdot(:,1);Rdot(:,2);Rdot(:,3);omegadot];
