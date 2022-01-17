function [u] = kinematic_lqr_controller(state, ref, param)
% KINEMATIC_LQR_CONTROLLER 
% This function calculates the control output to drive the ego vehicle.
% Inputs:
% state = [x, y, yaw, delta] is the ego state 
% ref = [x_ref; y_ref; yaw_ref; v_ref, curvature_ref] is the reference path
% param: model parameters
% Outputs: 
% u = [v_des, delta_des] is the control output

% calculate projection point from reference path as the closest point to
% ego
crosstrack_errors = vecnorm(ref(:,1:2)' - state(1:2)');
[~, min_index] = min(crosstrack_errors);
projection_point = ref(min_index, :);

% Throw error is ego goes out of bounds
assert(min_index < length(crosstrack_errors),'Ego ran out of path')
if min_index >=length(crosstrack_errors)
    return
end

% get reference values
v_des = projection_point(4);
k = projection_point(5);
projection_xy = projection_point(1:2);
yaw_ref =projection_point(3);
% ego state values 
ego_xy = state(1:2);
yaw = state(3);
v = v_des;

% coordinate transformation to body frame
Transform = [cos(yaw), sin(yaw);
    -sin(yaw), cos(yaw)];
error_vector_g_frame = (ego_xy - projection_xy)';
error_b_frame = Transform * error_vector_g_frame;

% get crosstrack and yaw error
lat_error = error_b_frame(2);
yaw_error = yaw - yaw_ref;
yaw_error = wrapToPi(yaw_error);

% compute LQ gains online. This can be done offline too. 
A = [0 v;
     0 0];
B = [0;
     ((v/param.wheelbase))];
Q = [0.01 0;
    0      1];
R= eye(1);
[K,~,~] = lqr(A,B,Q,R);
% feedforward calculation
delta_ff = atan2(param.wheelbase *k,1);
feedforward_control = [delta_ff];
feedback_control = -K*[lat_error;yaw_error];
total_delta = feedback_control+feedforward_control;

u = [v, total_delta];
end

