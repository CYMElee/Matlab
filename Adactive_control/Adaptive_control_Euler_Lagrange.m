clear all, close all , clc
%simulation time 10s
t=[1:1:10000];
dt=0.001;
g = 9.81;
%dynamic m*q_dd+m*g*l*sin(q)=T;
qd = 0;
q_dd = 0;
q_d  = 0;
q    = 1; %initial value
%The ground truth m=2,l=1,
m = 2;
l = 1;
x = 1;
%m_hat = 0;
%l_hat = 0;
%desire qd,qd_d
qd = 0;
qd_d = 0;
qd_dd = 0;
%define r & e
%e_d = q_d-qd_d;
%e   = q-qd;
a   = 3;
%r = e_d + a*e;
% define Y , Gama ,theta_hat ,theta_hat_dot ,k

Gama = 0.1;
%theta_hat = [m_hat ; m_hat*l_hat];
theta_hat = [0 ; 0];
%theta_hat_dot = Gama*Y*r;
k = 0.2;
%define control input T = Y.'*theta_hat+k*r;

%update
old_q_d = 0;
new_q_d = 0;

old_q = 0;
new_q = 0;

old_e = 0;
new_e = 0;

old_theta_hat=zeros(2,1);
new_theta_hat=zeros(2,1);


%record
record_q = zeros(length(t),1);
record_qd = zeros(length(t),1);
record_qd_d = zeros(length(t),1);
record_T = zeros(length(t),1);
record_theta_hat = zeros(2,length(t));
for i=1:length(t)
    record_q(i) = q;

    old_qd =qd;
    qd = sin(x) + cos(x/2);
    new_qd =qd;
    record_qd(i) = qd;
    
    old_qd_d =qd_d;
    new_qd_d = (new_qd-old_qd)/dt;
    qd_d=new_qd_d ;
    record_qd_d(i) = qd_d;
    
    
    qd_dd=(new_qd_d-old_qd_d)/dt;
    
    
    e = (qd-q);
    e_d = (qd_d-q_d)/dt;
    r = e_d + a*e;
    Y = [qd_dd+a*e_d ; g*sin(q)];
    
    T = (Y.')*theta_hat+k*r;
    record_T(i) = T;
    q_dd = (1/m)*(T-m*g*l*sin(q));
    %update
    theta_hat = theta_hat+(Gama*Y*r);
    record_theta_hat(:,i) = theta_hat;
    q_d = q_d + q_dd*dt;
    q = q + q_d*dt;  
    x=x+dt;
end

plot(t,record_q,'-m','LineWidth',3);
hold on;
plot(t,record_qd,':b','LineWidth',3);
legend('q',"qd");
xlim([0 10]);
ylim([-2 2]);
hold off;


