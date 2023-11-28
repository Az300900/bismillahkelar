% Control Initialization
% Renardi Adryantoro Priambudi
% Copyright 2022; Institut Teknologi Sepuluh Nopember.

%% add Image to the path
addpath(genpath('Images'));

%% load the scene data file generated from Driving Scenario Designer
load('test-scenario.mat');

%% define reference points
refPose = data.ActorSpecifications.Waypoints;
xRef = refPose(:,1);
yRef = -refPose(:,2);
waypoint = [xRef,yRef];
 
%% define reference time for plotting 
Ts = 400; % simulation time
s = size(xRef);
tRef = (linspace(0,Ts,s(1)))'; % this time variable is used in the "2D Visualization" block for plotting the reference points.

%% define parameters used in the models
L = 3; % bicycle length
ld = 4; % lookahead distance
X_o = refPose(1,1); % initial vehicle position
Y_o = -refPose(1,2); % initial vehicle position 
psi_o = 0; % initial yaw angle

%% Frenet Constant
refPathObj = referencePathFrenet(waypoint);
Sg = refPathObj.PathLength;
%frenetState = global2frenet(refPathObj,[positions(1),positions(2),0,0,0,0]);
%S = frenetState(1); %Travel Distance in path
%L = frenetState(4); %Lateral Distance in path
%Time_reference = 100;





