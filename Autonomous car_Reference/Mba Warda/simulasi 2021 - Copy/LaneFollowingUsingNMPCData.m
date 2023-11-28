

%% Parameters and curvature 
m = 1000;   % Mass of car
Iz = 2875;  % Moment of inertia about Z axis
lf = 1.2;   % Distance between Center of Gravity and Front axle 
lr = 1.6;   % Distance between Center of Gravity and Rear axle
Cf = 19000; % Cornering stiffness of the front tires (N/rad)
Cr = 33000; % Cornering stiffness of the rear tires (N/rad).
tau = 0.2;  % Time constant

%%
% Set the initial and driver-set velocities.
v0 = 10;    % Initial velocity
v_set = 30; % Driver set velocity

%%
% Set the controller sample time.
Ts = 0.1;

%%
% Obtain the lane curvature information.
% seconds.
% Duration = 20;                              % Simulation duration
% t = 0:Ts:Duration;                          % Time vector
% rho = LaneFollowingGetCurvature(v_set,t);   % Signal containing curvature information

curvature = load('dlcCurvatureNewV25.mat').curvature;
% curvature = xlsread('ujiPaperCurvature.xlsx');
% curvature = curvature(:,2)';
[a,b] = size(curvature);

Duration = b/10;                              % Simulation duration
t = 0:Ts:Duration-0.1;                          % Time vector
rho.time = t;
rho.signals.values = curvature'; 
