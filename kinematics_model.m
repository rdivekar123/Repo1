function x_dot = kinematics_model(state, control, param)
% This function calulates the derivate states which will used for forward
% propogating the ego state.
% Inputs:
% state = [x, y, yaw, delta] is the ego state 
% control = [v_desired, steering angle] is the control input to the model
% param: model parameters
% Outputs:
% x_dot: derivative of state 

v_desired = control(1);
delta_des = control(2);
% limit delta by the actuator limits
delta_des = max(min(delta_des, param.road_wheel_angle_limit), -param.road_wheel_angle_limit);
yaw = state(3);
delta = state(4);
v = v_desired;
x_dot = v * cos(yaw);
y_dot = v * sin(yaw);
yaw_dot = v * tan(delta) / param.wheelbase;
delta_dot = - (delta - delta_des) / param.tau;

x_dot = [x_dot, y_dot, yaw_dot, delta_dot];



