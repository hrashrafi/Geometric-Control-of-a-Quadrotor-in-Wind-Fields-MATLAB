function e_R= Gen_e_R(t,s)

m=1;
k_x=1;
k_v=1.4;
g=9.8;
b_1d=[1,0,0]';
x_d=[cos(2*t), zeros(length(t),2)];
xdot_d=[-2*sin(2*t), zeros(length(t),2)];
xdot2_d=[-4*cos(2*t), zeros(length(t),2)];

for i=1:length(t)
    
    e_x=s(i,1:3)'-x_d(i,1:3)';
    e_v=s(i,4:6)'-xdot_d(i,1:3)';
    A= -k_x*e_x-k_v*e_v-m*g*[0, 0, 1]'+m*xdot2_d(i,1:3)';
    
    % b_3 vector:
    
    b_3c=-A/norm(A);

    % b_2 vector:
    C=cross(b_1d,b_3c);
    n_C=norm(C);
    b_2c=-C/n_C;

    % b_1 vector:
    b_1c=cross(b_2c,b_3c);

    % Desired R:
    R_c=[b_1c,b_2c,b_3c];
    
    % Current R:
    R=[s(i,7:9)',s(i,10:12)',s(i,13:15)'];
    
    % Error Computation
    e_R_before_vee= .5*(R_c'*R-R'*R_c);
    e_R(i,1:3)= [e_R_before_vee(3,2), e_R_before_vee(1,3), e_R_before_vee(2,1)];
     
end