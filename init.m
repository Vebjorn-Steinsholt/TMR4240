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
Kp = diag([5e6   2e6   8e5]);
Ki = diag([1e4 1e4 1e4]);
Kd = diag([3e4 3e4 0]);

t_end = 1000;
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

% %% --- Example plots ---
% % 1) Ship states (12 states). Adjust labels to match your state ordering.
% figure('Name','Ship States (12x1)','NumberTitle','off');
% % for i = width(x_ship)
% plot(t_ship, X_ship(1,:), 'LineWidth', 1); grid on;
% xlabel('Time [s]');
% ylabel('State value');
% title('Ship States');
% legend(compose('x_%d',1:12), 'Location','bestoutside');
% 
% % 2) Environmental forces (12 components)
% figure('Name','Environmental Forces (12x1)','NumberTitle','off');
% plot(t_env, F_env, 'LineWidth', 1); grid on;
% xlabel('Time [s]');
% ylabel('Force / Moment');
% title('Environmental Forces');
% legend(compose('f_%d',1:12), 'Location','bestoutside');


%% Functions

% UI for choosing DP operation 
function [userDPMode, userCurrent, userWind] = simOptions()

f = figure('Position', [400, 200, 400, 600], ...
    'Name', 'DP operations', ...
    'MenuBar','none', ...
    'NumberTitle','off', ...
    'WindowStyle','modal');

% === DP objective (roomier, higher on the page) ===
bg1 = uibuttongroup('Parent', f, ...
    'Position', [0.07 0.62 0.86 0.30], ...   % moved up, a bit taller
    'Title', 'Select DP objective', ...
    'FontSize', 12, ...
    'FontWeight', 'bold');
obj1 = uicontrol(bg1, 'Style','radiobutton','FontSize',11, ...
    'String','Station Keeping', ...
    'Position',[12 95 300 24]);
obj2 = uicontrol(bg1, 'Style','radiobutton','FontSize',11, ...
    'String','Single Setpoint Path Following', ...
    'Position',[12 65 300 24]);
obj3 = uicontrol(bg1, 'Style','radiobutton','FontSize',11, ...
    'String','Multi Setpoint Path Following', ...
    'Position',[12 35 300 24]);

% === Current loads (centered vertically with even margins) ===
bg2 = uibuttongroup('Parent', f, ...
    'Position', [0.07 0.36 0.86 0.20], ...   % natural gap below bg1
    'Title', 'Select current loads', ...
    'FontSize', 12, ...
    'FontWeight', 'bold'); 
curr1 = uicontrol(bg2, 'Style','radiobutton','FontSize',11, ...
    'String','Zero current', ...
    'Position',[12 70 300 24]);
curr2 = uicontrol(bg2, 'Style','radiobutton','FontSize',11, ...
    'String','Const direction', ...
    'Position',[12 40 300 24]);
curr3 = uicontrol(bg2, 'Style','radiobutton','FontSize',11, ...
    'String','Var. direction', ...
    'Position',[12 10 300 24]);

% === Wind loads (above the OK button with comfortable spacing) ===
bg3 = uibuttongroup('Parent', f, ...
    'Position', [0.07 0.14 0.86 0.18], ...   % leaves space for OK button
    'Title', 'Select wind loads', ...
    'FontSize', 12, ...
    'FontWeight', 'bold'); 
wind1 = uicontrol(bg3, 'Style','radiobutton','FontSize',11, ...
    'String','Zero wind', ...
    'Position',[12 40 300 24]);
wind2 = uicontrol(bg3, 'Style','radiobutton','FontSize',11, ...
    'String','Const direction', ...
    'Position',[12 10 300 24]);

% OK button centered at the very bottom
uicontrol('Style','pushbutton', 'String','OK', 'FontSize',12, ...
    'Position',[150 15 100 40], ...          % moved down for symmetric look
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
