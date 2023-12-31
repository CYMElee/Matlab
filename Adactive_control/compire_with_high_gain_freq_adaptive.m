clear all, close all , clc
%dynamic is xdot =a^2*sin(x)+u here a is unknow(ground truth a =2)
%desing control input as u = -k1x-k2x

%the simulate time is 
t = [1:1:10000];
dt = 0.001;
% control gain k1,k2
k1=15;
k2=10;
%ground truth a
a=2;

%initial condiction x(0);
x=150;
%vector that using plot
record_xdot=zeros(length(t),1);
record_u=zeros(length(t),1);
%record_e=zeros(length(t),1);
record_x=zeros(length(t),1);
%the update term
state_before=0;
state_update=0;


%High gain control

for i =1:length(t)
    u=-k1*x-k2*x;
    record_u(i)=u;
    xdot=(a^2)*sin(x)+u;
    record_xdot(i)=xdot;
    record_u(i)=u;
    record_x(i)=x;
    state_before=x;
    state_update=state_before+xdot*dt;
    x=state_update;    
end
%plot 1
figure;
subplot(3,1,1);
plot(t,record_u,':b','LineWidth',3);
xlabel('Time');
ylabel('control input');
%xlim([0 100]);
%ylim([-500 150]);
hold on;
subplot(3,1,2);
plot(t,record_x,':b','LineWidth',3);
xlabel('Time');
ylabel('tracking errors');
%xlim([0 100]);
%ylim([-500 150]);
hold on;
%plot 2
figure;
subplot(3,1,1);
plot(t,record_x,'--g','LineWidth',3);
hold on;
plot(t,record_xdot,'-m','LineWidth',3);
title("high gain");
xlabel('Time');
xlim([0 400]);
ylim([-4000 150]);
legend('X',"Xd");
hold off;

%High frequency control
x = 150;
record_xdot=zeros(size(record_xdot));
record_u=zeros(size(record_u));
record_x=zeros(size(record_x));
state_before = 0;
state_update = 0;
for i =1:length(t)
    u=-k1*x-k2*sign(x);
    xdot=(a^2)*sin(x)+u;
    record_xdot(i)=xdot;
    record_u(i)=u;
    record_x(i)=x;
    state_before=x;
    state_update=state_before+xdot*dt;
    x=state_update;    
end
%plot 1
figure(1);
subplot(3,1,1);
plot(t,record_u,'*c','LineWidth',3);
xlabel('Time');
ylabel('control input');
%xlim([0 100]);
%ylim([-500 150]);
hold on;
subplot(3,1,2);
plot(t,record_x,'*c','LineWidth',3);
xlabel('Time');
ylabel('tracking errors');
%xlim([0 100]);
%ylim([-500 150]);
hold on;

%plot 2
figure(2);
subplot(3,1,2);
plot(t,record_x,'--g','LineWidth',3);
hold on;
plot(t,record_xdot,'-m','LineWidth',3);
title("high frequency");
xlabel('Time');
xlim([0 400]);
ylim([-2500 150]);
legend('X',"Xd");
hold off;

%Adaptive control
x = 150;
record_xdot=zeros(size(record_xdot));
record_u=zeros(size(record_u));
record_x=zeros(size(record_x));
state_before=0;
state_update=0;
theta_before=0;
theta_update=0;
%defing theta_estimate assume the initial_condiction of theta_hat is 0;
theta_hat = 0;
% let a^2 =theta
theta = a^2;
%design theta_hat_dot as x*sin(x)
for i =1:length(t)
    u=-theta_hat*sin(x)-k1*x;
    xdot=(theta)*sin(x)+u;
    record_xdot(i)=xdot;
    record_u(i)=u;
    record_x(i)=x;
    %renew x
    state_before=x;
    state_update=state_before+xdot*dt;
    x=state_update;
    
    % renew theta_hat
    theta_before = theta_hat;
    theta_update = theta_before + x*sin(x)*dt;
    theta_hat = theta_update;
    
end
%plot 1
figure(1);
subplot(3,1,1);
plot(t,record_u,'--r','LineWidth',3);
xlabel('Time');
ylabel('control input');
legend("high_gain_u","high_freq_u","adaptive_u");
xlim([0 650]);
ylim([-2500 10]);
hold off;
subplot(3,1,2);
plot(t,record_x,'--r','LineWidth',3);
xlabel('Time');
ylabel('tracking errors');
xlim([0 500]);
ylim([-50 200]);
legend("high_gain_e","high_freq_e","adaptive_e");
hold off;

%plot 2
figure(2);
subplot(3,1,3);
plot(t,record_x,'--g','LineWidth',3);
hold on;
plot(t,record_xdot,'-m','LineWidth',3);
title("adaptive control");
xlabel('Time');
xlim([0 400]);
ylim([-2500 150]);
legend('X',"Xd");
hold off;



