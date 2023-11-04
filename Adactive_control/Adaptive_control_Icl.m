% system dynamics xdot = a^2*sin(x)+u
% desire position xd = cos(4t)
% let theta = a^2 theta_hat  theta_tilde
% e=x-xd
% design control input u = - theta_hat*sin(x)-k*e+4*(-sin(4*t(i)))
% design CL: we choice kcl as 5,gamma as same as 5, and we only consider the last 20 data 

% assum x(0)=0,a=5

%we design adaptive_law as xsin(x)
% initial value

%assume_our_cl_turn is  
a = 5;
b = 0:0.001:5;
for o=1:5001
    b(o) = a^2;
end
k = 5.0;
kcl = 25.0;
Gamma = 20;
record =zeros(1,21);
delt = 0.01;
x = 0:0.01:50;
e = 0:0.01:50;
theta_hat = 0:0.01:50;
theta_hat(1)=2;
theta_tilda = 0:0.01:50;
t=0:0.01:50;
xd=cos(4*t);
buffer =0;



for i = 1:5000
    e(i) = x(i) - xd(i);
    u =4*(-sin(4*t(i)))-(theta_hat(i))*(sin(x(i)))-k*e(i);
    if i == 1
           N = 0;
           CL = 0;
    else
       ui =  (4*(-sin(4*t(i)))-(theta_hat(i))*sin(x(i))-k*e(i))*delt;
       yi = sin(x(i))*delt;
       record(N) = sin(x(i))*(x(i)-x(i-1)- ui - yi*theta_hat(i));
        if N > 1 && N < 20
            for u=1:N
            record(N+1) = record(N+1) + record(u);
            end
            CL = kcl*Gamma*record(N+1);
        elseif N == 20
            for u=0:19
            record(N+1) = record(N) + record(N-u);
            end
            CL = kcl*Gamma*record(N+1);
        else
            CL = kcl*Gamma*record(N);
        end
        
        
    end
    theta_tilda(i) = (a^2)-theta_hat(i);
    theta_hat(i+1) = theta_hat(i) + (Gamma*(sin(x(i)))*e(i)+CL)*delt;
    x_dot = (a^2)*(sin(x(i)));
    x(i+1) = x(i)+(x_dot+u)*delt;
    if N<20
    N = N+1;
    else
    end
    
end
error =1:1:5001;
for p=1:5001
error(p) = (theta_tilda(p)/abs(theta_hat(p)))*100;
end
subplot(1,3,1);
plot(t,xd,'--g',t,x,'-m');
subplot(1,3,2);
plot(t,b,'-g',t,theta_hat,'-m');
subplot(1,3,3);
plot(t,error,'-g');