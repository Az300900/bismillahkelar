% Car Dynamics Simulation with PID Control, Pure Pursuit, and Speed PID
% Renardi Adryantoro Priambudi
% Copyright 2022-2023; Institut Teknologi Sepuluh Nopember.

clc;
clear;
close all;

% Car properties
mass = 1000;                % Mass of the car (kg)
initial_position = [0, 0];  % Initial position of the car (x, y)
initial_speed = 0;          % Initial speed of the car (m/s)
initial_steering_angle = 0; % Initial steering angle of the car (radians)
L = 2;                      % Length of the car (m)

% Car obstacle
obstacle1=[35,35];
obstacle2=[250,6];
%obstacle3=[500,-15];
obstacle4=[756,12];
obstacles = [obstacle1;obstacle2; obstacle4];
radius=4;

% Simulation time span
total_time = 60;            % Total simulation time (seconds)
time_step = 0.1;            % Simulation time step (seconds)
num_steps = total_time / time_step;
time_span = linspace(0, total_time, num_steps);

% Simulation time span path
total_time_path = 10;
time_span_path = linspace(0, total_time_path, num_steps);
desired_distance = 1000;

%Time Mission Control Variable
tr = total_time; % Time Reference

% Generate trajectory with the specified path distance
predefined_trajectory = predefined_trajectory_positions(time_span_path, desired_distance);

% Calculate the curvature for each segment of the path
trajectory_radius = predefined_trajectory(:, 2:3);
curvature = zeros(size(trajectory_radius, 1), 1);
for i = 4:numel(trajectory_radius(:,1))-1
    % Get the coordinates of the current segment
    x1 = trajectory_radius(i-2, 1);
    y1 = trajectory_radius(i-2, 2);
    x2 = trajectory_radius(i, 1);
    y2 = trajectory_radius(i, 2);

    % Check if it is the last point in the trajectory
    if i == numel(trajectory_radius)-1
        x3 = trajectory_radius(end, 1);
        y3 = trajectory_radius(end, 2);
    else
        x3 = trajectory_radius(i+1, 1);
        y3 = trajectory_radius(i+1, 2);
    end
    
    curvature(i) = evaluate_curvature([x1,y1;x2,y2;x3,y3]);
    
end

% Plot the curvature and radius along the path
figure;
plot(curvature, 'b.-', 'LineWidth', 2);
xlabel('Segment');
ylabel('Curvature 1/m');
title('Curvature along the Path');
grid on;


% Get PATH distance
% Calculate the path distance
path_distance = 0;
for i = 2:size(predefined_trajectory, 1)
    path_distance = path_distance + norm(predefined_trajectory(i, :) - predefined_trajectory(i-1, :));
end



%Generate velocity profile
[speed_reference_no_curvature,position_reference] = velocity_profile(time_span,0,tr,path_distance);
speed_reference_no_curvature(end+1)=0;
speed_reference = speed_reference_no_curvature;
speed_reference2 = speed_reference_no_curvature;

%linearization Speed
flag=1;
%{
for i = 1:numel(trajectory_radius(:,1))
    %Linearization
    threshold=13;
    in_min=8;
    in_max=12;
    out_max = threshold-threshold*0.2;
    out_min = threshold;
    curvature_s = abs(curvature(i)*1000);

   if curvature_s>=8
        if speed_reference(i) >= 13
            speed_reference(i) = map(curvature_s, in_min, in_max, out_min, out_max);
            speed_reference(i+1) = speed_reference(i);
            speed_reference_temp =speed_reference(i); 
        end
   else
       %Calculate Distance leftover
       path_leftover_distance = calculate_distance_from_path(trajectory_radius(i:end,:));
       time_left_over = tr - time_span(i);
       
   end
end
%}
%disp(size(speed_reference_no_curvature))
%disp(size(speed_reference))




%% Start Program
% Define the differential equation
car_dynamics = @(t, y) car_ode(t, y, mass, @input_speed, @pure_pursuit, @pid_steering, tr, @pid_speed, predefined_trajectory,speed_reference,position_reference,time_span,obstacles,curvature, L);

% Initial condition
initial_conditions = [initial_position, initial_speed, initial_steering_angle];

% Solve the differential equation
[t, y] = ode45(car_dynamics, time_span, initial_conditions);

% Extract position, speed, and steering angle from the solution
positions = y(:, 1:2);
speeds = y(:, 3);
steering_angles = y(:, 4);

% Get the trajectory positions
% trajectory_positions = trajectory(t);
trajectory_positions = predefined_trajectory(:,2:3);

%% PLOT Necesarry variable
% PLOT Global Path
figure
plot(predefined_trajectory(:, 2)/1000, predefined_trajectory(:, 3)/1000, '--','LineWidth', 2);
hold off;
xlabel('X position (Km)');
ylabel('Y position (Km)');
title(sprintf('Car Global Path | Length: %d Km', int64(path_distance/1000)));
legend('Desired Trajectory');
grid on;

% PLOT Reference Speed no curvature
figure
plot(t, speed_reference_no_curvature * 3.6);
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Car Speed Reference_no_curvature vs. Time');
grid on;

% PLOT Reference Speed
figure
plot(t, speed_reference * 3.6);
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Car Speed Reference vs. Time');
grid on;

% PLOT Reference Speed
%figure
%plot(t, speed_reference2 * 3.6);
%xlabel('Time (s)');
%ylabel('Speed (km/h)');
%title('Car Speed Reference vs. Time');
%grid on;

% Plot the position vs. time with trajectory
figure;
plot(positions(:, 1), positions(:, 2),'LineWidth', 2);
hold on;
plot(trajectory_positions(:, 1), trajectory_positions(:, 2), '--','LineWidth', 2);

draw_obstacle(gca, obstacle1,radius)
draw_obstacle(gca, obstacle2,radius)
%draw_obstacle(gca, obstacle3,radius)
draw_obstacle(gca, obstacle4,radius)

hold off;
xlabel('X position (m)');
ylabel('Y position (m)');
title('Car Position vs. Time with Trajectory');
legend('Car Position', 'Desired Trajectory');
grid on;


% Plot the speed vs. time
figure;
plot(t, speeds * 3.6);
xlabel('Time (s)');
ylabel('Speed (km/h)');
title('Car Speed vs. Time');
grid on;


% Plot the steering angle vs. time
%figure;
%plot(t, rad2deg(steering_angles));
%xlabel('Time (s)');
%ylabel('Steering Angle (Deg)');
%title('Car Steering Angle vs. Time');
%grid on;


%% Solver
% Initialize the lookahead points array
lookahead_points = [];

% Car ODE function with PID control
function dydt = car_ode(t, feed_back, mass, input_speed, pure_pursuit, pid_steering, tr, pid_speed, predefined_trajectory, speed_reference,position_reference,time_span,obstacles,curvature,L)
    position = feed_back(1:2);
    speed = feed_back(3);
    steering_angle = feed_back(4);

    % Get the input values
    % Bezier Active
    path = generate_path(position,predefined_trajectory,obstacles,speed);
    %path = predefined_trajectory(:,2:3);

    % Steering
    steering_input = pure_pursuit(position,path);
    
    % Limit the steering angle to 30 degrees
    % max_steering_angle = deg2rad(30);
    % steering_input = max(min(steering_input, max_steering_angle), -max_steering_angle);
    
    steering_error = steering_input - steering_angle;
    steering_angle_input = pid_steering(steering_error);
    
    %Speed'
    speed_input = input_speed(t,speed_reference,time_span,curvature,tr,position,path);
    speed_error = speed_input - speed;
    throttle_input = pid_speed(speed_error);

    % Kinematic bicycle model
    dx = speed * cos(steering_angle);
    dy = speed * sin(steering_angle);
    dg = speed / L * tan(steering_angle_input);
    ds = throttle_input / mass;

    % Calculate the change in position, speed, and steering angle
    position_change = [dx; dy];
    speed_change = ds;
    steering_angle_change = dg;

    % Derivatives of position, speed, and steering angle
    dydt = [position_change; speed_change; steering_angle_change];
    
end

% Speed input function
function speed_input = input_speed(t,speed_reference,time_span,curvature,tr,position,path)
    % Example: Constant speed of 20 m/s
    % Speed_input = 20;

    % Find the index corresponding to the desired time value
    [~, index] = min(abs(t - time_span));
    
       
        %New algorithm Failed
        % Position in path
        index_path = find_closest_index(position,path);
        path_leftover_distance = calculate_distance_from_path(path(index_path:end,:));
        time_left_over = tr - time_span(index);
        
        %Linearization
        threshold=16.6;
        in_min=0.008;
        in_max=0.012;
        out_max = threshold-threshold*0.2;
        out_min = threshold;
        curvature_s = abs(curvature(index));
        
           if curvature_s>=0.008
                if speed_reference(index) >= threshold %46 km/h
                    speed_reference(index) = map(curvature_s, in_min, in_max, out_min, out_max);
                end
           else
               
             if speed_reference(index) >= threshold %60 km/h
                 if path_leftover_distance < 900 && path_leftover_distance > 100
                   speed_compensation = (path_leftover_distance/time_left_over);
                   speed_compensation = speed_compensation + speed_compensation*0.2;
                   speed_reference(index) = speed_compensation;
                 end

                  if path_leftover_distance < 10
                       speed_reference(index) = 0;
                  end
             end

           end

   speed_input = speed_reference(index);
end




function [speed_reference,q]= velocity_profile(t,to,tr,path_distance)
    t0 = to;
    tf = tr;
    q0=0;
    qf=path_distance;

    V=1.05*(qf-q0)/tf;
    tb=(q0-qf+V*tf)/V;

    ind1=(t0<=t) & (t<=tb);
    ind2=(tb<t) & (t<=tf-tb);
    ind3=(tf-tb<=t) & (t<=tf);
    
    q1=q0+V*t.^2/(2*tb);
    q2=(qf+q0-V*tf)/2+V*t;
    q3=qf-V*tf^2/(2*tb)+V*tf*t/tb - V*t.^2/(2*tb);
    q = [q1(ind1) q2(ind2) q3(ind3)];

    size_q=size(q);
    size_q=size_q(2);
    size_t=size(t);
    size_t=size_t(2);

    if (size_q > size_t)
        q(end)=[];
    elseif (size_q < size_t)
        q(end+1)=path_distance;
    end
    qdot=diff(q)./diff(t);
    speed_reference = qdot;
end


% PID control for steering
function target_point = pure_pursuit(position, trajectory)
    % Pure Pursuit
    lookahead_distance = 2; % Lookahead distance in meters

    % Calculate the distances from the car to all trajectory points
    distances = sqrt((trajectory(:, 1) - position(1)).^2 + (trajectory(:, 2) - position(2)).^2);
    
    valid_indices = distances <= lookahead_distance;
    if any(valid_indices)
        % Find the index of the trajectory point closest to the lookahead distance
        index = find(distances <= lookahead_distance, 1, 'last');
        % Get the coordinates of the lookahead point
        lookahead_point = trajectory(index, :);
    else
        lookahead_point = trajectory(end, :);
    end

    % Calculate the desired steering angle using the Pure Pursuit algorithm
    target_point = atan2(lookahead_point(2) - position(2), lookahead_point(1) - position(1));
    
end

% PID control for steering
function steering_angle_input = pid_steering(error)
    persistent integral
    persistent prev_error

    % PID control parameters for steering
    kp_steering = 0.5;     % Proportional gain for steering
    ki_steering = 0.0;  % Integral gain for steerung
    kd_steering = 0.0;  % Derivative gain for steering
    
    % Initialize persistent variables on the first call
    if isempty(integral)
        integral = 0;
        prev_error = 0;
    end

    % Calculate the control signal components
    proportional = kp_steering * error;
    integral = integral + ki_steering * error;
    derivative = kd_steering * (error - prev_error);
    
    % Calculate the throttle input
    steering_angle_input = proportional + integral + derivative;
    
    % Update previous error for the next iteration
    prev_error = error;
end
% PID control for speed
function throttle_input = pid_speed(error)
    persistent integral
    persistent prev_error

    % PID control parameters for speed
    %kp_speed = 100;   % Proportional gain for speed
    %ki_speed = 80;  % Integral gain for speed
    %kd_speed = 0.1;  % Derivative gain for speed

    %kp_speed = 20;   % Proportional gain for speed
    %ki_speed = 10;  % Integral gain for speed
    %kd_speed = 0.0;  % Derivative gain for speed

    kp_speed = 1000;   % Proportional gain for speed
    ki_speed = 0.0;  % Integral gain for speed
    kd_speed = 0.0;  % Derivative gain for speed

    %kp_speed = 100;   % Proportional gain for speed
    %ki_speed = 80;  % Integral gain for speed
    %kd_speed = 0.1;  % Derivative gain for speed
    
    % Initialize persistent variables on the first call
    if isempty(integral)
        integral = 0;
        prev_error = 0;
    end

    % Calculate the control signal components
    proportional = kp_speed * error;
    integral = integral + ki_speed * error;
    derivative = kd_speed * (error - prev_error);
    
    % Calculate the throttle input
    throttle_input = proportional + integral + derivative;
    
    % Update previous error for the next iteration
    prev_error = error;
end

% Predefined Trajectory function
function predefined_trajectory_positions = predefined_trajectory_positions(t,desired_distance)
    % Define the desired trajectory
    x = t;
    y = sin(t);

    % Rescale the trajectory to match the desired distance
    actual_distance = calculate_path_distance(x, y);
    scaling_factor = desired_distance / actual_distance;
    x = x * scaling_factor;
    y = y * scaling_factor;

    predefined_trajectory_positions = [t', x', y']; % Combine x, and y positions
end
% Function to calculate the path distance
function distance = calculate_path_distance(x, y)
    distance = sum(sqrt(diff(x).^2 + diff(y).^2));
end


function result = map(x, in_min, in_max, out_min, out_max)
    result = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    %result(result < out_min) = out_min;
    %result(result > out_max) = out_max;
end
function curve_points = generate_path(position,predefined_trajectory,obstacles,speed)
    % Checking Which Obstacle
    closest_obstacle = find_closest_obstacle(position, obstacles);
    % Calculate the distances from the car to all trajectory points
    distance = sqrt((closest_obstacle(:, 1) - position(1)).^2 + (closest_obstacle(:, 2) - position(2)).^2);
    
    %Prediction Control
    safe_distance = map(speed, 0,28,10,50);

    if distance < safe_distance
        predefined_path = predefined_trajectory(:,2:3);
        keypoints = generate_keypoint(predefined_path,closest_obstacle,safe_distance);
       
        distances = sqrt(sum((predefined_path(:,:) - keypoints(1,:)).^2, 2));
        [~, closest_index_1] = min(distances);
        distances = sqrt(sum((predefined_path(:,:) - keypoints(end,:)).^2, 2));
        [~, closest_index_end] = min(distances);
        
        sliced_predifined_path = predefined_path(closest_index_1:closest_index_end, :);
        curve = generate_bezier_point(keypoints,size(sliced_predifined_path));
        % Indices for insertion
        startIdx = closest_index_1;
        endIdx = closest_index_end;
        % Get the portion of globalpath after the insertion point
        tail = predefined_path(endIdx+1:end, :);
        % Concatenate the head of globalpath, generatedpath, and the tail
        curve_points = [predefined_path(1:startIdx-1, :); curve; tail];
    else
        curve_points = predefined_trajectory(:,2:3);
    end
end

function points = generate_bezier_point(control_points, number_of_points)
    number_of_points = number_of_points(1);
    % Calculate the parameter values based on the number of points
    t = linspace(0, 1, number_of_points);
    
    % Calculate points on the cubic Bezier curve
    P0 = control_points(1, :);
    P1 = control_points(2, :);
    P2 = control_points(3, :);
    P3 = control_points(4, :);
    
    points = zeros(number_of_points, 2);  % Initialize array for points
    
    for i = 1:number_of_points
        points(i, :) = (1 - t(i)).^3 * P0 + 3 * (1 - t(i)).^2 * t(i) * P1 + 3 * (1 - t(i)) * t(i).^2 * P2 + t(i).^3 * P3;
    end
end



function closest_obstacle = find_closest_obstacle(position, obstacles)
    % Find the closest obstacle to the given position
    min_distance = Inf;
    closest_obstacle = [];

    for i = 1:size(obstacles, 1)
        obstacle = obstacles(i, :);
        obstacle_distance = norm(obstacle - position);

        if obstacle_distance < min_distance
            min_distance = obstacle_distance;
            closest_obstacle = obstacle;
        end
    end
end

function keypoints = generate_keypoint(path, center,radius)
    point1 = find_point_with_radius(path,center,radius );
    % Given points
    point2 = center;
    angle_increment = 0; % Angle increment in degrees
    % Initialize the array to store the generated points
    additional_points = zeros(3, 2);
    % Calculate the angle of the vector from point1 to point2
    vector = point2 - point1;
    angle_rad = atan2(vector(2), vector(1));
    
        for i = 1:3
            angle = angle_rad + deg2rad(angle_increment); % Calculate the angle
            x = point2(1) + radius * cos(angle);
            y = point2(2) + radius * sin(angle);
            additional_points(i, :) = [x, y]; % Store the coordinates
            angle_increment=angle_increment+60;
        end
        additional_points(end+1,:)=point1;
        keypoints = additional_points;
end


function desired_point = find_point_with_radius(predefined_path, start_point, radius)
    % Find the closest point on the predefined path to the start point
    [~, start_index] = min(vecnorm(predefined_path - start_point, 2, 2));

    distance_accumulator = 0;
    for i = start_index+1 : size(predefined_path, 1)
        % Calculate the distance between the current point and the previous point
        distance = norm(predefined_path(i, :) - predefined_path(i-1, :));
        
        % Add the distance to the distance accumulator
        distance_accumulator = distance_accumulator + distance;
        
        % Check if the distance accumulator is greater than or equal to the desired radius
        if distance_accumulator >= radius
            % Calculate the position of the desired point on the path
            t = (radius - (distance_accumulator - distance)) / distance;
            desired_point = (1 - t) * predefined_path(i-1, :) + t * predefined_path(i, :);
            break;
        end
    end
  
end

function curvature = evaluate_curvature(positions)
    x1 = positions(1,1);
    y1 = positions(1,2);
    x2 = positions(2,1);
    y2 = positions(2,2);
    x3 = positions(end,1);
    y3 = positions(end,2);
    % Calculate the coefficients of the circle's equation
    A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;
    B = (x1^2 + y1^2) * (y3 - y2) + (x2^2 + y2^2) * (y1 - y3) + (x3^2 + y3^2) * (y2 - y1);
    C = (x1^2 + y1^2) * (x2 - x3) + (x2^2 + y2^2) * (x3 - x1) + (x3^2 + y3^2) * (x1 - x2);

    % Calculate the coordinates of the circle's center
    center_x = -B / (2 * A);
    center_y = -C / (2 * A);

    % Calculate the radius of the circle
    radius = sqrt((x1 - center_x)^2 + (y1 - center_y)^2);

    % Calculate the curvature as the reciprocal of the radius
    %curvature(i) = 1 / radius;
    curvature = sign((x3 - x1) * (y2 - y1) - (y3 - y1) * (x2 - x1)) * 1 / radius;
end

function closest_index = find_closest_index(position, path)
    % Calculate the distances from the position to all points on the path
    distances = sqrt((path(:, 1) - position(1)).^2 + (path(:, 2) - position(2)).^2);
    
    % Find the index of the closest point
    [~, closest_index] = min(distances);
end




function path_distance=calculate_distance_from_path(path)
    path_distance = 0;
    for i = 2:size(path, 1)
        path_distance = path_distance + norm(path(i, :) - path(i-1, :));
    end
end





function draw_obstacle(axesHandle,position,radiusMeters)
    color = 'r';  % Color for filling the circle
    plotFilledCircle(axesHandle, position, radiusMeters, color)
end
function plotFilledCircle(axesHandle, center, radiusMeters, color)
    % Set the given figure as the current figure
    % Set the given axes as the current axes
    axes(axesHandle);

    % Define the angle range for plotting the circle
    theta = linspace(0, 2*pi, 100);

    % Calculate the circle points
    x = center(1) + radiusMeters * cos(theta);
    y = center(2) + radiusMeters * sin(theta);

    % Plot the filled circle
    fill(x, y, color, 'LineWidth', 2);
    axis equal;  % Set equal aspect ratio for x and y axes
    grid on;  % Display grid lines

    % Add label for the radius
    text(center(1) + radiusMeters, center(2), sprintf('%d meter', radiusMeters), 'HorizontalAlignment', 'left');
end

%% ARCHIVE
function trajectory_data = trajectory(t)
 % Define the desired trajectory
    x = t;
    y = sin(t);
    trajectory_data = [x', y']; % Combine x, and y positions
end
