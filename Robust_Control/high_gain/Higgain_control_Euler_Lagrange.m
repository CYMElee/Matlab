clear all, close all , clc
%simulation time 10s
t=[1:1:10000];
dt=0.001;
t_plot=[0:dt:9.999];
g = 9.81;
%The ground truth m=2,l=1,a=2
m = 2;
l = 1;
a = 1;
k = 5;
k2 =20;
c1 = 5;
%the desire q,qd_d,qd_dd
qd = 0;
qd_d =0;
qd_dd = 0;



% the error term
e = (1-3);
e_d =0;
e_dd = 0;
r = 0;
%the dynamic
q_dd = 0;
q_d =0;
q = 3; %initial q = 0

%control input T
T = 0;
% update term
old_q = 0;
new_q = 0;
old_q_d = 0;
new_q_d = 0;


%record
record_q = zeros(length(t),1);
record_qd = zeros(length(t),1);

for i=1:length(t)
    record_q(i) = q;
    record_qd(i) =qd;
    record_e(i)=e;
    old_qd =qd;
    new_qd = sin(i*dt)+cos((i*dt)/2); 
    qd=new_qd;
    old_qd_d = qd_d;
    new_qd_d = (new_qd-old_qd)/dt;
    qd_d = new_qd_d;
    e = (qd-q);
    e_d =(qd_d-q_d);
    r = e_d+a*e;
    T=k*r+k2*c1*r;
    q_dd =-g*l*sin(q)+T/m;
    q_d = q_d+(q_dd*dt);
    q = q + (q_d*dt);
  
end


tiledlayout(2,1);
nexttile
plot(t_plot,record_q,'-g','LineWidth',3);
title("Desire vs Measure");
hold on;
plot(t_plot,record_qd,':b','LineWidth',3);
legend('q',"qd");
xlim([0 10]);
ylim([-10 10]);
xlabel('Time');
ylabel('Response');
hold off;
nexttile
plot(t_plot,record_e,'-g','LineWidth',3);
title("Error");
legend('e');
xlabel('Time');
ylabel('error');
