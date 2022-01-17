function [outputStates] = simulate(model, controller, x0, ref, end_time, param)
% This function forward propogates the ego states for the given simulation
% time.
% Inputs: 
% model: Mathematical model function handle used to forward simulate the motion of ego
% controller: function handle to the controller which will calculate the
% control action
% x0: init state of the ego
% ref: reference path
% end_time: Length of simulation in seconds
% param: Model params
% Outputs: 
% outputStates: Log of all the output states from ego motion. 
x = x0;
t = 0;
dt = param.control_dt;
i=1;
outputStates = zeros(length(0:dt:end_time)-1, length(x0));
while t <end_time
    u = controller(x, ref, param);
    x_dot = model(x, u, param);
    x = x + x_dot * dt;
    outputStates(i,:) = x;
    i=i+1;
    t=t+dt;
    % uncomment following lines for animation 
%     fig1 = figure(1);
%     plot(ref(:,1), ref(:,2))
%     hold on
%     plot(x(1), x(2), 'bo');
%     pause(0.005)
%     clf
end
hold off
end

