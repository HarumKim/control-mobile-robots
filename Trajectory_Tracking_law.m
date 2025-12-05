function [v,w] = Trajectory_Tracking_law( ...
    x, y, theta, ...
    xr, yr, xdr, ydr, xddr, yddr, ...
    vr, wr, ...
    v_prev, Ts, ...
    kp1, kp2, kd1, kd2, ...
    k1, k2, k3, ...
    ctrl_type)

% TRAJECTORY_TRACKING_LAW

% Inputs:
%   (x,y,theta)   : current robot state
%   xr,yr         : position reference
%   xdr,ydr       : velocity reference (dxr/dt, dyr/dt)
%   xddr,yddr     : acceleration reference (d2xr/dt2, d2yr/dt2)
%   vr,wr         : unicycle reference velocities (linear and angular)
%   v_prev        : previous linear velocity (not used here, but kept for
%                   compatibility with the base code)
%   Ts            : sampling time
%   kp*,kd*       : PD gains for the FL+PD controller
%   k1,k2,k3      : gains for the De Luca controller

% Outputs:
%   v,w           : linear and angular velocity commands

% --------- Reference orientation ---------
thetar = atan2(ydr, xdr);  % trajectory heading

% --------- Position errors in global coordinates ---------
ex = xr - x;
ey = yr - y;

% --------- Error transformation to robot frame ---------
% (e1, e2): position error in robot frame
%  e3     : orientation error
e1 =  cos(theta)*ex + sin(theta)*ey;
e2 = -sin(theta)*ex + cos(theta)*ey;

e3 = thetar - theta;
% wrap to [-pi, pi] to avoid large jumps
e3 = atan2(sin(e3), cos(e3));

% --------- For FL+PD we need e1_dot, e2_dot ---------
persistent e1_prev e2_prev
if isempty(e1_prev)
    e1_prev = e1;
    e2_prev = e2;
end

e1_dot = (e1 - e1_prev)/Ts;
e2_dot = (e2 - e2_prev)/Ts;

% update memory
e1_prev = e1;
e2_prev = e2;

% --------- Controller selection ---------
switch ctrl_type
    
    case 1  % ================= De Luca ==========================
        %
        % Classic De Luca law (simplified form):
        %   v = vr * cos(e3) + k1 * e1
        %   w = wr + k2 * e2 + k3 * sin(e3)
        %
        v = vr * cos(e3) + k1 * e1;
        w = wr + k2 * e2 + k3 * sin(e3);
        
    case 2  % ================= FL + PD ==========================
        %
        % Feedback linearization + PD:
        % virtual inputs on (e1, e2):
        %   u1 = kp1*e1 + kd1*e1_dot
        %   u2 = kp2*e2 + kd2*e2_dot
        %
        % superimposed on the feedforward reference:
        %   v = vr * cos(e3) + u1
        %   w = wr + u2
        %
        u1 = kp1 * e1 + kd1 * e1_dot;
        u2 = kp2 * e2 + kd2 * e2_dot;
        
        v = vr * cos(e3) + u1;
        w = wr + u2;
        
    otherwise
        error('ctrl_type must be 1 (De Luca) or 2 (FL+PD).');
end

end
