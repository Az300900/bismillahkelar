%RENARDI ADRYANTORO PRIAMBUDI
%6022211030

%% Velocity Profile
clc;
clear;
close all;

t0 = 0;
tf = 1000;
q0=0;
qf=1000;

V=1.5*(qf-q0)/tf;
tb=(q0-qf+V*tf)/V;

time_step = 0.1;            % Simulation time step (seconds)
num_steps = tf / time_step;
t = linspace(t0, tf, num_steps); % Specify the desired t values

ind1=find((t0<=t) & (t<=tb));
ind2=find((tb<t) & (t<=tf-tb));
ind3=find((tf-tb<=t) & (t<=tf));

q1=q0+V*t.^2/(2*tb);
q2=(qf+q0-V*tf)/2+V*t;
q3=qf-V*tf^2/(2*tb)+V*tf*t/tb - V*t.^2/(2*tb);
q = [q1(ind1) q2(ind2) q3(ind3)];
%q = cast(q,"uint8");

disp(size(q1))
disp(size(q2))
disp(size(q3))
disp(size(q))

qdot=diff(q)./diff(t);
q2dot=diff(qdot)./diff(t(1:end-1));



figure(3);
subplot(3,1,1)
plot(t,q/1000);
xlabel('t(sec)')
ylabel('q(t) (Km)')
title("Velocity Profile")

subplot(3,1,2)
plot(t(1:end-1),qdot*3.6)
xlabel('t (sec)')
ylabel('dq/dt(t) (Km/h)')

subplot(3,1,3);
plot(t(1:end-2),q2dot)
xlabel('t (sec)')
ylabel('d^2q/dt^2(t) (m/s^2)')






