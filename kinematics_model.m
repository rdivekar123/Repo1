function x_dot = kinematics_model(state, control, param)
% state = [x, y, yaw, delta]
% control = [v_des, delta_des]

v_des = control(1);
delta_des = control(2);
delta_des = max(min(delta_des, param.road_wheel_angle_limit), -param.road_wheel_angle_limit);
yaw = state(3);
delta = state(4);
v = v_des;
x_dot = v * cos(yaw);
y_dot = v * sin(yaw);
yaw_dot = v * tan(delta) / param.wheelbase;
delta_dot = - (delta - delta_des) / param.tau;

x_dot = [x_dot, y_dot, yaw_dot, delta_dot];



