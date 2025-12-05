%% Main_control_code.m
% Trajectory tracking simulation for a differential–drive robot
% using two controllers:
%   - De Luca (ctrl_type = 1)
%   - Feedback Linearization + PD (ctrl_type = 2)

clear all; close all; clc;

%% Robot parameters (QBot)
r = 0.04445;  % wheel radius [m]
d = 0.393;    % distance between wheels [m]

M    = [[r/2, r/2]; [r/d, -r/d]]; % Transformation matrix
Minv = inv(M);                    % Inverse transformation matrix

% Sampling time
Ts = 0.15;

%% Initial conditions
x0 = 0; 
y0 = 0; 
theta0 = 0;
q0 = [x0; y0; theta0];  % initial state

%% Reference trajectory
eta   = 1;   % scaling factor
alpha = 4;
k     = 0:Ts:2*pi*alpha*2;

xr   = eta * sin(k/alpha);          % reference position in x
yr   = eta * sin(k/(2*alpha));      % reference position in y

% Trajectory velocities
xdr  = eta * cos(k/alpha)      * (1/alpha);
ydr  = eta * cos(k/(2*alpha))  * (1/(2*alpha));

% Trajectory accelerations
xddr = -eta * sin(k/alpha)     * (1/alpha)^2;
yddr = -eta * sin(k/(2*alpha)) * (1/(2*alpha))^2;

% Trajectory orientation
thetar = atan2(ydr, xdr);

% "Unwrapping" of thetar to avoid large jumps
thetar_diff = diff(thetar);
i1 = 1; i2 = length(thetar);
for i = 1:length(thetar_diff)
    if thetar_diff(i) < -6
        i1 = i+1;
    elseif thetar_diff(i) > 6
        i2 = i;
    end
end
thetar(i1:i2) = thetar(i1:i2) + 2*pi;

% Unicycle reference velocities
vr = sqrt(xdr.^2 + ydr.^2);
wr = (yddr .* xdr - xddr .* ydr) ./ (xdr.^2 + ydr.^2 + 1e-6); % avoid division by zero

% Trajectory length
Kf = length(xr);

%% Controller gains

% --- De Luca controller gains ---
k1 = 1.5;
k2 = 2.0;
k3 = 1.0;

% --- Feedback Linearization + PD gains ---
kp1 = 2.0;
kp2 = 2.0;
kd1 = 0.3;
kd2 = 0.3;

%% Input constraints (wheel speeds)
wrmax = 10;     % right wheel limit (rad/s)
wlmax = wrmax;  % left wheel limit

%% Simulation – CONTROLLER 1: De Luca (ctrl_type = 1)

ctrl_type = 1;

qseq1  = q0;       % state sequence
vseq1  = [];       % v(k)
wseq1  = [];       % w(k)

% Initialize v with the first reference linear velocity
v = vr(1);

for K = 1:Kf
    
    % Current state
    x     = qseq1(1,end);
    y     = qseq1(2,end);
    theta = qseq1(3,end);
    
    % De Luca control law
    [v, w] = Trajectory_Tracking_law( ...
        x, y, theta, ...
        xr(K), yr(K), ...
        xdr(K), ydr(K), ...
        xddr(K), yddr(K), ...
        vr(K), wr(K), ...
        v, Ts, ...
        kp1, kp2, kd1, kd2, ...
        k1, k2, k3, ...
        ctrl_type);
    
    % Unicycle saturation
    [v, w] = unicycle_saturation(wrmax, wlmax, v, w, r, d);
    
    % Store inputs
    vseq1 = [vseq1, v];
    wseq1 = [wseq1, w];
    
    % Integrate differential-drive model (ode45) in [0, Ts]
    tspan = [0, Ts];
    [~, q] = ode45(@(t,q) DiffDrive(t,q,v,w), tspan, qseq1(:,end));
    
    % Update state sequence
    qseq1 = [qseq1, q(end,:)'];
end

t_sim = 0:Ts:(Kf-1)*Ts;   % time vector for Kf points

% Use the first Kf states to compare with the reference
x1     = qseq1(1,1:Kf);
y1     = qseq1(2,1:Kf);
theta1 = qseq1(3,1:Kf);

%% Simulation – CONTROLLER 2: Feedback Linearization + PD (ctrl_type = 2)

ctrl_type = 2;

qseq2  = q0;
vseq2  = [];
wseq2  = [];

v = vr(1);

for K = 1:Kf
    
    x     = qseq2(1,end);
    y     = qseq2(2,end);
    theta = qseq2(3,end);
    
    % FL + PD control law
    [v, w] = Trajectory_Tracking_law( ...
        x, y, theta, ...
        xr(K), yr(K), ...
        xdr(K), ydr(K), ...
        xddr(K), yddr(K), ...
        vr(K), wr(K), ...
        v, Ts, ...
        kp1, kp2, kd1, kd2, ...
        k1, k2, k3, ...
        ctrl_type);
    
    % Saturation
    [v, w] = unicycle_saturation(wrmax, wlmax, v, w, r, d);
    
    vseq2 = [vseq2, v];
    wseq2 = [wseq2, w];
    
    tspan = [0, Ts];
    [~, q] = ode45(@(t,q) DiffDrive(t,q,v,w), tspan, qseq2(:,end));
    
    qseq2 = [qseq2, q(end,:)'];
end

x2     = qseq2(1,1:Kf);
y2     = qseq2(2,1:Kf);
theta2 = qseq2(3,1:Kf);

%% Tracking error

ex1 = xr - x1;
ey1 = yr - y1;
e_norm1 = sqrt(ex1.^2 + ey1.^2);

ex2 = xr - x2;
ey2 = yr - y2;
e_norm2 = sqrt(ex2.^2 + ey2.^2);

%% Comparison plots

% ---- Trajectories ----
figure;
plot(xr, yr, 'k--', 'LineWidth', 1.5); hold on;
plot(x1, y1, 'b',  'LineWidth', 1.5);
plot(x2, y2, 'r',  'LineWidth', 1.5);
grid on; axis equal;
xlabel('x [m]');
ylabel('y [m]');
title('Reference trajectory vs. tracked trajectories');
legend('Reference','De Luca','FL + PD','Location','Best');

% ---- Position error norm ----
figure;
plot(t_sim, e_norm1, 'b', 'LineWidth', 1.5); hold on;
plot(t_sim, e_norm2, 'r', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('||e|| = sqrt(e_x^2 + e_y^2) [m]');
title('Position tracking error');
legend('De Luca','FL + PD','Location','Best');

% ---- Control inputs v(t) and w(t) ----
figure;
subplot(2,1,1);
plot(t_sim, vseq1, 'b', 'LineWidth', 1.2); hold on;
plot(t_sim, vseq2, 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('v [m/s]');
title('Linear velocity v(t)');
legend('De Luca','FL + PD','Location','Best');

subplot(2,1,2);
plot(t_sim, wseq1, 'b', 'LineWidth', 1.2); hold on;
plot(t_sim, wseq2, 'r', 'LineWidth', 1.2);
grid on;
xlabel('Time [s]');
ylabel('\omega [rad/s]');
title('Angular velocity \omega(t)');
legend('De Luca','FL + PD','Location','Best');
