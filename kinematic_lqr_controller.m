function [u] = kinematic_lqr_controller(state, ref, param)
% KINEMATIC_LQR_CONTROLLER 
% state = [x, y, yaw, delta]
% ref = [x_ref; y_ref; yaw_ref; v_ref, curvature_ref];
% control = [v_des, delta_des]

crosstrack_errors = vecnorm(ref(:,1:2)' - state(1:2)');
[~, min_index] = min(crosstrack_errors);
projection_point = ref(min_index, :);
v_des = projection_point(4);
k = projection_point(5);

assert(min_index < length(crosstrack_errors),'Ego ran out of path')
if min_index >=length(crosstrack_errors)
    return
end
yaw = state(3);
v = v_des;

% coordinate transformation to body frame
Transform = [cos(yaw), sin(yaw);
    -sin(yaw), cos(yaw)];
error_vector_g_frame = (state(1:2) - projection_point(1:2))';
error_b_frame = Transform * error_vector_g_frame;
lat_error = error_b_frame(2);
yaw_error = yaw - projection_point(3);
yaw_error = wrapToPi(yaw_error);
delta = state(4);

A = [0 v;
     0 0];
B = [0;
     ((v/param.wheelbase))];
Q = [0.01 0;
    0      1];
R= eye(1);
[K,~,~] = lqr(A,B,Q,R);
% feedforward input calculation
delta_ff = atan2(param.wheelbase *k,1);
feedforward_control = [delta_ff];
feedback_control = -K*[lat_error;yaw_error];
total_delta = feedback_control+feedforward_control;
u = [v, total_delta];
end
