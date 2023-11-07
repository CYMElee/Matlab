clear all, close all , clc

%simulate time 10s
t=[1:1:10000];
dt=0.001;
x=150;

%the open loop dynamic is x'=a*sin(x)+u a=5
a=5;
%we design feedback linearization controller as 
%u=-asin(x)-kx, here we choice k as 15
%assune I.C x(0)=100
k=15;
%record xdot and x
record_xdot=zeros(length(t),1);
record_x=zeros(length(t),1);
%the update term
state_before=0;
state_update=0;

for i=1:length(t)
    u=-a*sin(x)-k*x;
    xdot=a*sin(x)+u;
    record_xdot(i)=xdot;
    record_x(i)=x;
    state_before=x;
    state_update=state_before+xdot*dt;
    x=state_update;
    
end   
plot(t,record_x,'--g','LineWidth',3);
hold on;
plot(t,record_xdot,'-m','LineWidth',3);
title("feedback linearization");
xlabel('Time');
xlim([0 100]);
ylim([-500 150]);

legend('X',"Xd");

