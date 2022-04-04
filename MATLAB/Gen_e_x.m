function e_x= Gen_e_x(t,s)

x_d=[cos(2*t), zeros(length(t),2)];

e_x=s(:,1:3)-x_d;



