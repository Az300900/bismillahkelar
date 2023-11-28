
%% Clear All Data
clc; clear; close all
% specify simulation stop time
%% Overview of Simulink Model
% Open the Simulink model.
mdl = 'LaneFollowingNMPCWarda';
% mdl = 'LaneFollowingNMPC';

open_system(mdl)

%% NLMPC Model Input and Output
nlobj = nlmpc(7,3,'MV',[1 2],'MD',3,'UD',4);

%% Time, Horizon Control
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 30;
nlobj.ControlHorizon = 2;

%% State Function and Jacobian State Function
nlobj.Model.StateFcn = @(x,u) LaneFollowingStateFcn(x,u);
nlobj.Jacobian.StateFcn = @(x,u) LaneFollowingStateJacFcn(x,u);

%% Output Function for the Nonlinear Plant Model
nlobj.Model.OutputFcn = @(x,u) [x(3);x(5);x(6)+x(7)];
nlobj.Jacobian.OutputFcn = @(x,u) [0 0 1 0 0 0 0;0 0 0 0 1 0 0;0 0 0 0 0 1 1];

%% Set the constraints for manipulated variables.
nlobj.MV(1).Min = -3;      % Maximum acceleration 3 m/s^2
nlobj.MV(1).Max = 3;       % Minimum acceleration -3 m/s^2
nlobj.MV(2).Min = -0.6;   % Minimum steering angle -65 
nlobj.MV(2).Max = 0.6;    % Maximum steering angle 65

%% Set the scale factors.
nlobj.OV(1).ScaleFactor = 15;   % Typical value of longitudinal velocity
nlobj.OV(2).ScaleFactor = 1;  % Range for lateral deviation
nlobj.OV(3).ScaleFactor = 0.5;  % Range for relative yaw angle
nlobj.MV(1).ScaleFactor = 6;    % Range of steering angle
nlobj.MV(2).ScaleFactor = 2.26; % Range of acceleration
nlobj.MD(1).ScaleFactor = 0.2;  % Range of Curvature

%% Specify the weights in the standard MPC cost function.
nlobj.Weights.OutputVariables = [1 1 0];

%% Penalize acceleration change more for smooth driving experience.
nlobj.Weights.ManipulatedVariablesRate = [0.4 0.2];

%% Validate prediction model functions at an arbitrary operating point using
% the |validateFcns| command. At this operating point:
% * |x0| contains the state values.
% * |u0| contains the input values.
% * |ref0| contains the output reference values.
% * |md0| contains the measured disturbance value.
% x0 = [0.1 0.5 5 0.1 0.1 0.001 0.5];
x0 = [0.01 0.01 10 0.01 0.01 0.001 10];
u0 = [0.125 0.4];
ref0 = [30 0 0];
md0 = 0.001;
validateFcns(nlobj,x0,u0,md0,{},ref0);

%% Run Simulink Simulation
sim(mdl);
logsout1 = logsout;

%% Plot and compare simulation results.


% % Plot Acceleration
% 
% figure; 
% plot(logsout1{2}.Values.Time,logsout1{2}.Values.Data);
% title('NLMPC Acceleration');
% xlabel('Time(s)');
% ylabel('Acceleration(m/s^2)');
% 
% % Plot Steering Angle
% 
% figure; 
% plot(logsout1{3}.Values.Time,logsout1{3}.Values.Data);
% title('NLMPC Steering Angle');
% xlabel('Time(s)');
% ylabel('rad');
% 
% % Plot Longitudinal Velocity
% 
% figure; 
% plot(logsout1{19}.Values.Time,logsout1{19}.Values.Data);
% title('NLMPC Longitudinal Velocity');
% xlabel('Time(s)');
% ylabel('Velocity(m/s)');
% 
% % Plot Lateral Deviation
% 
figure; 
plot(logsout1{14}.Values.Time,logsout1{14}.Values.Data);
title('NLMPC Lateral Deviation');
xlabel('Time(s)');
ylabel('Value of Error(m)');
% 
% % Plot Relative Yaw Angle 
% 
figure; 
plot(logsout1{15}.Values.Time,logsout1{15}.Values.Data);
title('NLMPC Yaw Deviation');
xlabel('Time(s)');
ylabel('Value of Error(radian)');
% 
% % Plot Postion Result 
% % 
% figure; 
% plot(logsout1{8}.Values.Data,logsout1{9}.Values.Data);
% title('NLMPC x position vs y position');
% xlabel('X(m)');
% ylabel('Y(m)');
% 
% % Plot Acceleration - PID
% 
figure; 
plot(logsout1{10}.Values.Time,logsout1{10}.Values.Data);
title('cascade Acceleration');
xlabel('Time(s)');
ylabel('Acceleration(m/s^2)');
% 
% % Plot Steering Angle - PID
% 
figure(10); 
plot(logsout1{11}.Values.Time,logsout1{11}.Values.Data);
title('cascade Steering Angle');
xlabel('Time(s)');
ylabel('rad');
% 
% % Plot Longitudinal Velocity - PID
% 
figure(11) 
plot(logsout1{20}.Values.Time,logsout1{20}.Values.Data);
title('cascade Longitudinal Velocity');
xlabel('Time(s)');
ylabel('Velocity(m/s)');
% %plot dua kecepatan
% figure(23)
% plot(logsout1{19}.Values.Time,logsout1{19}.Values.Data);hold on;
% plot(logsout1{22}.Values.Time,logsout1{22}.Values.Data);
% legend('NMPC','Cascade')
% xlabel('X(m)')
% ylabel('Y(m)')
% % 
% % Plot Lateral Deviation - PID
% 
figure(12)
plot(logsout1{12}.Values.Time,logsout1{12}.Values.Data);
title('cascade Lateral Deviation');
xlabel('Time(s)');
ylabel('Value of Error(m)');
% 
% % Plot Relative Yaw Angle - PID
% 
figure(13)
plot(logsout1{13}.Values.Time,logsout1{13}.Values.Data);
title('cascade Yaw Deviation');
xlabel('Time(s)');
ylabel('Value of Error(radian)');

