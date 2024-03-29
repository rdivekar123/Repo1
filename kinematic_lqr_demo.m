clc;
clear all;

% Design path in global frame 
x_ref = 1:1:199;
y_ref = zeros (1,199);

x_ref_back = 199:-1:1;
y_ref_back = 100*ones (1,199);
% path = [[x_ref]' [y_ref]'];
% Semicircle center
x=200;      
y=50;
r=50;
theta = linspace(-pi/2, pi/2, 100);  
x_semicircle = r * cos(theta) + x;
y_semicircle = r * sin(theta) + y;
path = [[x_ref x_semicircle x_ref_back]' [y_ref, y_semicircle y_ref_back]'];
plot(path(:, 1), path(:,2))

%% reference line generation
x_spline = csaps(1:1:length(path),path(:,1),0.1,1:0.01:length(path));
y_spline = csaps(1:1:length(path),path(:,2),0.1,1:0.01:length(path));
% f = figure()
% plot(x_spline, y_spline)

% calculate yawref
yaw_spline = calculateYawref(x_spline, y_spline);

xy_spline = [x_spline', y_spline'];

% insert Meneger curvature
curvature_spline = calculateCurvature(xy_spline);

% insert velocity
v_spline = 15.0 *ones(1, length(x_spline));

ref_line = [x_spline', y_spline', yaw_spline', v_spline', curvature_spline'];
%%
% vehicle is offset from path      
% state vector is [x , y , yaw, delta]
x0 = [2.0, 10.0 , pi/6, 0.0];

%%
% parameters for kinematic model
param.wheelbase = 2.9;
param.control_dt = 0.01; % [s]
param.road_wheel_angle_limit = 30 * 180/pi;
param.tau = 0.05;
%% simulate model with controller in loop 
simulation_time = 30.0;
X = simulate(@kinematics_model, @kinematic_lqr_controller, x0, ref_line, simulation_time, param);

%% Plot Reference and Ego Path 
fig = figure(2);
plot(x_spline, y_spline, 'r');
axis equal;
hold on;
plot(X(:,1),X(:,2), 'b');
legend('reference path','ego path')
grid on 
hold off;

function yaw_ref = calculateYawref(x,y)
% calculateYawref Local function calculates yaw angles from given x, y path.
yaw_ref = zeros(1, length(x));
for i = 2:length(x)-1
    x_next = x(i+1);
    x_prev = x(i-1);
    y_next = y(i+1);
    y_prev = y(i-1);
    yaw_ref(i) = atan2(y_next-y_prev, x_next-x_prev);
end
yaw_ref(1) = yaw_ref(2);
yaw_ref(end) = yaw_ref(end-1);
end

function curvature_ref = calculateCurvature(xy_path)
% calculateYawref Local function calculates yaw angles from given xy path.
curvature_ref = zeros(1, length(xy_path));
for i = 2:length(xy_path)-1
    point1 = xy_path(i-1,:);
    point2 = xy_path(i,:);
    point3 = xy_path(i+1,:);
    area = ((point2(1)-point1(1))*(point3(2)-point1(2)) - (point2(2)-point1(2))*(point3(1)-point1(1)))/2;
    curvature_ref(i) = 4 * area / (norm(point1-point2) * norm(point2-point3) * norm(point3-point1));
end
curvature_ref(1) = curvature_ref(2);
curvature_ref(end) = curvature_ref(end-1);
end