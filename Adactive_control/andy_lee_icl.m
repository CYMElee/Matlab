clear all, close all , clc

t = [1:1:10000];
dt = 0.01;

% array for recording the states (for plot)
record_X = zeros(length(t),1);
record_Xd = zeros(length(t),1);
adaptive_input = zeros(length(t),1);
record_theta_hat = zeros(length(t),1);
record_theta = zeros(length(t),1);

% ICL record
record_Y = zeros(length(t),1);
record_x = zeros(length(t),1);
record_u = zeros(length(t),1);


k1 = 6;
a = 2;
x = 0;
u = 0;
theta_hat_1 = 5;
gamma = 3;
x_before = 0;
kcl = 4;
% The adaptive control 

for i=1:length(t)
    theta = a*a;
    xd = sin(i*dt);
    xd_dot = cos(i*dt);
    
    dynamics_update = x - x_before;
    x_before = x;

    % Regression Matrix
    Y = sin(x);
    
    record_X(i) = x;
    record_Xd(i) = xd;
    record_x(i) = dynamics_update;
    record_theta_hat(i) = theta_hat_1;
    record_theta(i) = theta;

    e = x - xd;

    record_u(i) = u*dt;
    record_Y(i) = Y*dt;
    icl_term = 0;

    % when time > 5s (0.01*500), ICL starts to work
    if i > 100
        % Assume N = 100
        for j = 1:10
            icl_term = icl_term + record_Y(i-j)*(record_x(i-j) - record_u(i-j) - record_Y(i-j)*theta_hat_1);
        end
        
    end

    % estimate parameters
    theta_hat_dot_1 = gamma*Y*e + kcl*gamma*icl_term;
    theta_hat_1 = theta_hat_1 + theta_hat_dot_1*dt;

    % control input
    u = xd_dot - k1*e - Y*theta_hat_1;

    x_dot = theta*Y + u;
    x = x + x_dot*dt;

end


figure(1);
tiledlayout(2,1);
nexttile
% Plot tracking error
plot(t,record_X,t,record_Xd) 
title("Tracking errors");
legend('X',"Xd")
nexttile

% Plot input signal 
plot(t,record_theta,t,record_theta_hat);
title("Theta");
legend('Theta','Theta Hat','Location','southeast')