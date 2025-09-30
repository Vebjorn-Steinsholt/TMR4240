%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init()                                                                  %
%                                                                         %              
% Set initial parameters for part1.slx and part2.slx                      %
%                                                                         %
% Created:      2018.07.12	Jon Bjørnø                                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear; clc; close all;

load('thrusters_sup.mat')
load('supply.mat');
load('supplyABC.mat');

[dpMode, currentMode, windFlag] = simOptions();

SimulationParam
windCoefficients


% Initial position x, y, z, phi, theta, psi
eta0 = [0,0,0,0,0,0]';
% Initial velocity u, v, w, p, q, r
nu0 = [0,0,0,0,0,0]';

windAngle = 0;

% PID parameters
Kp = diag([7.8e5   9.8e5   1.5e8]);
Ki = diag([8.5e3 7.5e3 3.5e4]);
Kd = diag([9.3e5 7e5 4e6]);

t_end = 500;
dt = 0.01;

[xd_setpoint, t] = setPointGen(dpMode, t_end, dt);

simin = timeseries(xd_setpoint(1:3,:),t);

%% --- Run Simulink model and extract outputs ---
mdl = 'part1';

% Ensure Simulink uses variables from this script's workspace
simOut = sim(mdl, 'SrcWorkspace','current');

% Extract simulation outputs
t_ship = simOut.tout;
X_ship = reshape(simOut.ship_states, 12, [])';
F_env = simOut.env_forces;

%% Plots
% Extract ship states (positions & heading)
x_ship   = X_ship(:,1);   % surge position (x)
y_ship   = X_ship(:,2);   % sway position (y)
psi_ship = X_ship(:,6);   % heading (yaw)

u_ship   = X_ship(:,7);   % surge rate (u)
v_ship   = X_ship(:,8);   % sway rate (v)
r_ship   = X_ship(:,12);  % yaw rate (r)

% Extract reference from xd_setpoint 
x_ref_full   = xd_setpoint(1,:).';
y_ref_full   = xd_setpoint(2,:).';
psi_ref_full = xd_setpoint(3,:).';

u_ref_full  = xd_setpoint(4,:).'; 
v_ref_full  = xd_setpoint(5,:).';
r_ref_full  = xd_setpoint(6,:).';

% Interpolate position/heading reference onto t_ship
x_ref   = interp1(t, x_ref_full,   t_ship, 'linear', 'extrap');
y_ref   = interp1(t, y_ref_full,   t_ship, 'linear', 'extrap');
psi_ref = interp1(t, psi_ref_full, t_ship, 'linear', 'extrap');

% Interpolate velocity/yaw-rate references onto t_ship
u_ref = interp1(t, u_ref_full, t_ship, 'linear', 'extrap');
v_ref = interp1(t, v_ref_full, t_ship, 'linear', 'extrap');
r_ref = interp1(t, r_ref_full, t_ship, 'linear', 'extrap');

% Unwrap headings to prevent 2 jumps, and also provide degrees
psi_ship_u   = unwrap(psi_ship);
psi_ref_u    = unwrap(psi_ref);
psi_ship_deg = rad2deg(psi_ship_u);
psi_ref_deg  = rad2deg(psi_ref_u);

% Convert yaw rates to deg/s for readability (matches your psi in deg)
r_ship_deg = rad2deg(r_ship);
r_ref_deg  = rad2deg(r_ref);

% Choose plot limits from DP mode
if dpMode == 1
    posXLim = [-1, 1]; posYLim = [-1,1]; headingLim = [-1.5, 1.5];
elseif dpMode == 2
    posXLim = [-1, 12]; posYLim = [-1, 12]; headingLim = [-140, 60];
elseif dpMode == 3
    posXLim = [-10, 60]; posYLim = [-60, 10]; headingLim = [-60, 60];
elseif dpMode == 4
    posXLim = [-1, 18]; posYLim = [-1, 15]; headingLim = [-140, 60];
else
    error('select valid DP mode');
end

% Position and heading: surge (x), sway (y), yaw rate (psi)
% Surge (x)
figure; subplot(3,1,1)
plot(t_ship, x_ship, 'LineWidth',1.5); hold on;
plot(t_ship, x_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('x [m]'); title('Surge (position)');
legend('Ship','Reference','Location','best');
ylim(posXLim);

% Sway (y)
subplot(3,1,2)
plot(t_ship, y_ship, 'LineWidth',1.5); hold on;
plot(t_ship, y_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('y [m]'); title('Sway (position)');
legend('Ship','Reference','Location','best');
ylim(posYLim);

% Heading (psi)
subplot(3,1,3);
plot(t_ship, psi_ship_deg, 'LineWidth',1.5); hold on;
plot(t_ship, psi_ref_deg,  '--', 'LineWidth',1.5);
grid on; ylabel('$\psi$ [deg]'); xlabel('Time [s]'); title('Heading');
legend('Ship','Reference','Location','best');
ylim(headingLim);

% Rates: surge (u), sway (v), yaw rate (r)
figure; 

% Surge rate (u)
subplot(3,1,1)
plot(t_ship, u_ship, 'LineWidth',1.5); hold on;
plot(t_ship, u_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('u [m/s]'); title('Surge rate');
legend('Ship','Reference','Location','best');
ylim([-2.5, 2.5]);

% Sway rate (v)
subplot(3,1,2)
plot(t_ship, v_ship, 'LineWidth',1.5); hold on;
plot(t_ship, v_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('v [m/s]'); title('Sway rate');
legend('Ship','Reference','Location','best');

% Yaw rate (r)
subplot(3,1,3)
plot(t_ship, r_ship_deg, 'LineWidth',1.5); hold on;
plot(t_ship, r_ref_deg,  '--', 'LineWidth',1.5);
grid on; ylabel('r [deg/s]'); xlabel('Time [s]'); title('Yaw rate');
legend('Ship','Reference','Location','best');
ylim([-1.5, 1.5]);

% --- XY plot (plan view): ship trajectory vs reference trajectory ---
figure('Name','XY Trajectory (Surge vs Sway)','Color','w');
plot(y_ship, x_ship, 'LineWidth',1.5); hold on;
plot(y_ref,  x_ref,  '--', 'LineWidth',1.5);
grid on; axis equal;
xlabel('y [m]'); ylabel('x [m]');
title('Plan View: x-y Trajectory');
legend('Ship','Reference','Location','best');
xlim(posYLim); ylim(posXLim);

% Optional: start markers and heading arrows
plot(x_ship(1), y_ship(1), 'o', 'MarkerSize',6, 'HandleVisibility','off');
plot(x_ref(1),  y_ref(1),  'o', 'MarkerSize',6, 'HandleVisibility','off');

%% Functions

function [xd,t] = setPointGen(spModeIn, T_final, timeStep)
%SETPOINTGEN Summary of this function goes here
%   Detailed explanation goes here

% Generate set-points for simulation tests
spMode = spModeIn; % 1: [0 0 0], 2: single waypoint, 3: multi waypoint, 4: No ref. model

% Position and attitude ref model, based on Fossen, 2021
T11 = 50; T22 = 60; T33 = 80;

omega11 = 2*pi/T11; omega22 = 2*pi/T22; omega33 = 2*pi/T33; % natural freq.
zeta11 = 1; zeta22 = 1; zeta33 = 1;               % relative damping coeff.

Omega = diag([omega11 omega22 omega33]);
Delta = diag([zeta11 zeta22 zeta33]);

% Third order state-space representation
Ad = [zeros(3)                       eye(3)                   zeros(3);
      zeros(3)                     zeros(3)                   eye(3);
      -Omega^3 -(2.*Delta + eye(3))*Omega^2 -(2.*Delta + eye(3))*Omega];

Bd = [zeros(3) zeros(3) Omega^3]';

f_pa = @(xd, r) Ad*xd + Bd*r;

% Simulation parameters
dt = timeStep;          % Time step
t_final = T_final;      % Final time
t = 0:dt:t_final;   % Time vector
N = length(t);

xd0 = [0 0 0 0 0 0 0 0 0]';   % Initial states
% xd = [eta_d eta_d_dot eta_d_ddot]';
% eta_d = [x y psi]';
% eta_d_dot = [u v r]';
% eta_d_ddot = [u_dot v_dot r_dot];

% Set points reference model
switch spMode
    case 1
        waypoints = [0 0 0]';
    case 2
        waypoints = [10 10 ssa((3*pi)/2)]'; 
    case 3
        eta_sp0 = [0 0 0]';
        eta_sp1 = [50 0 0]';
        eta_sp2 = [50 -50 0]'; 
        eta_sp3 = [50 -50 ssa(-pi/4)]';
        eta_sp4 = [0 -50 ssa(-pi/4)]';
        eta_sp5 = [0 0 0]';
        waypoints = [eta_sp0 eta_sp1 eta_sp2 eta_sp3 eta_sp4 eta_sp5];
    case 4 
        waypoints = [10 10 ssa((3*pi)/2)]';
        xd0(1) = waypoints(1); xd0(2) = waypoints(2); xd0(3) = waypoints(3);
    otherwise
        disp('Choose a valid DP mode');
end

N_wp = size(waypoints, 2);
wp_interval = floor(N/N_wp-1);



xd = zeros(9, N);   % State trajectory
xd(:,1) = xd0;      % Initial condition

% Main simulation loop
for k = 1:N-1
    wp_idx = min(floor((k-1)/wp_interval)+1, N_wp);
    r = waypoints(:,wp_idx);

    xd(:,k+1) = rk4(@(x) f_pa(x, r), dt, xd(:,k));
end

end

% UI for choosing DP operation 
function [userDPMode, userCurrent, userWind] = simOptions()

f = figure('Position', [400, 200, 400, 600], ...
    'Name', 'DP operations', ...
    'MenuBar','none', ...
    'NumberTitle','off', ...
    'WindowStyle','modal');

% DP objective 
bg1 = uibuttongroup('Parent', f, ...
    'Position', [0.07 0.62 0.86 0.30], ...
    'Title', 'Select DP objective', ...
    'FontSize', 12, ...
    'FontWeight', 'bold');
obj1 = uicontrol(bg1, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Station Keeping', ...
    'Position',[12 95 300 24]);
obj2 = uicontrol(bg1, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Single Setpoint Path Following', ...
    'Position',[12 65 300 24]);
obj3 = uicontrol(bg1, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Multi Setpoint Path Following', ...
    'Position',[12 35 300 24]);
obj4 = uicontrol(bg1, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Without ref. model', ...
    'Position',[12 10 300 24]);

% Current loads 
bg2 = uibuttongroup('Parent', f, ...
    'Position', [0.07 0.36 0.86 0.20], ...
    'Title', 'Select current loads', ...
    'FontSize', 12, ...
    'FontWeight', 'bold'); 
curr1 = uicontrol(bg2, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Zero current', ...
    'Position',[12 70 300 24]);
curr2 = uicontrol(bg2, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Const direction', ...
    'Position',[12 40 300 24]);
curr3 = uicontrol(bg2, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Var. direction', ...
    'Position',[12 10 300 24]);

% Wind loads 
bg3 = uibuttongroup('Parent', f, ...
    'Position', [0.07 0.14 0.86 0.18], ...
    'Title', 'Select wind loads', ...
    'FontSize', 12, ...
    'FontWeight', 'bold'); 
wind1 = uicontrol(bg3, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Zero wind', ...
    'Position',[12 40 300 24]);
wind2 = uicontrol(bg3, ...
    'Style','radiobutton', ...
    'FontSize',11, ...
    'String','Const direction', ...
    'Position',[12 10 300 24]);

% OK button centered at the bottom
uicontrol('Style','pushbutton', 'String','OK', 'FontSize',12, ...
    'Position',[150 15 100 40], ...
    'Callback', @(src, evt) uiresume(f));

uiwait(f);

% Determine DP objective
if ~ishandle(bg1) || ~ishandle(bg2)
    userDPMode = 1;     
    userCurrent = 1;    
    userWind = 1;       
    try delete(f); end
    return
end

sel1 = bg1.SelectedObject;
if isequal(sel1, obj1)
    userDPMode = 1;
elseif isequal(sel1, obj2)
    userDPMode = 2;
elseif isequal(sel1, obj3)
    userDPMode = 3;
elseif isequal(sel1, obj4)
    userDPMode = 4;
else
    userDPMode = 1;
end

sel2 = bg2.SelectedObject;
if isequal(sel2, curr1)
    userCurrent = 1;
elseif isequal(sel2, curr2)
    userCurrent = 2;
elseif isequal(sel2, curr3)
    userCurrent = 3;
else
    userCurrent = 1;
end

sel3 = bg3.SelectedObject;
if isequal(sel3, wind1)
    userWind = 1;
elseif isequal(sel3, wind2)
    userWind = 2;
else
    userWind = 1;
end

if ishandle(f), delete(f); end
end
