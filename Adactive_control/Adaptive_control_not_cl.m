% system dynamics xdot = a^2*sin(x)+u
% desire position xd = cos(4t)
% let theta = a^2 theta_hat  theta_tilde
% e=x-xd
% design control input u = - theta_hat*sin(x)-k*e+4*(-sin(4*t(i)))

% assum x(0)=0,a=5

%we design adaptive_law as xsin(x)
% initial value

%assume_our_cl_turn is  
a = 5;
b = 0:0.01:50;
for o=1:5001
    b(o) = a^2;
end
k = 5.0;
Gamma = 20;
delt = 0.01;
x = 0:0.01:50;
e = 0:0.01:50;
theta_hat = 0:0.01:50;
t=0:0.01:50;
xd=cos(4*t);


for i = 1:5000
    e(i) = x(i) - xd(i);
    u =4*(-sin(4*t(i)))-(theta_hat(i))*(sin(x(i)))-k*e(i);
    theta_hat(i+1) = theta_hat(i) + (Gamma*(sin(x(i)))*e(i))*delt;
    x_dot = (a^2)*(sin(x(i)));
    x(i+1) = x(i)+(x_dot+u)*delt;
   
    
end
subplot(1,2,1);
plot(t,xd,'--g',t,x,'-m');
subplot(1,2,2);
plot (t,b,'-g',t,theta_hat,'-m');