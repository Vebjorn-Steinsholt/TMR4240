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

[dpMode, observerMode, currentMode, windMode, waveFlag, ctrlFlag] = simOptions();

SimulationParam
windCoefficients


% Initial position x, y, z, phi, theta, psi
eta0 = [0,0,0,0,0,0]';
% Initial velocity u, v, w, p, q, r
nu0 = [0,0,0,0,0,0]';

windAngle = 0;
%Thrust allocation starting point

alpha0 = [pi/2; 0; pi/2; -pi/6; pi/6];
f_max = [125; 150; 125; 300; 300] * 1000;
f0 = f_max/1000;
thruster_config = [1 1 1 1 1];

% PID controller
[Kp, Ki, Kd, M_pid] = init_pid(vessel);



t_end = 100;
h = 0.1;

%Non-linear Observer
% Mass matrix (M = MRB + MA)


% Damping matrix (D)
D = [2.6486e5     0           0;
     0        8.8164e5       0;
     0             0     3.3774e8];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = vesselABC.MA + vesselABC.MRB;
idx = [1 2 6];
M= M(idx, idx);
M_inv = inv(M);


%Wave freqcontrol plant model
%frequency
omega_mat = [pi/10 0 0;
            0 pi/10 0;
            0 0 pi/10];
%damping ration
zeta_mat = [1 0 0;
            0 1 0;
            0 0 1];
%system matrix
Aw = [zeros(3) eye(3);
    -omega_mat^2 -2*zeta_mat*omega_mat];
%Measurement matrix
Cw = [zeros(3) eye(3)];
%Bias time constat matrix
T_mat = [1000 0 0;
        0 1000 0;
        0 0 1000];
T_inv = T_mat\eye(3);
%Tuning
%frequency and damping ratios, ni:deciered, i: actual

zeta_ni = 1.1;
zeta_i = 0.1;
w_i = pi/9*2;
w_ci = 1*pi;
%diagonla values
k11 = -2*(zeta_ni-zeta_i)*w_ci/w_i;
k12 = 2*w_i*(zeta_ni-zeta_i);
k2 = w_ci;
%Observer Gains matrixes
K1 = [k11 0 0;
      0 k11*2 0;
      0 0 k11;
      k12 0 0;
      0 k12 0;
      0 0 k12;]*4;
K2 = [k2 0 0;
      0 k2 0;
      0 0 k2];
K4 = M*0.1;
K3 = K4*0.2;


%Harris wind spectrum
% parameters
dt = 0.001;
t = 0:dt:500;

Kappa = 0.0026; 
L = 1800; 
U_bar = 10;

f = linspace(0.001, 0.01, 200);
df = f(2)-f(1);                       
phaseAngle = 2*pi*rand(1, numel(f));

% Harris spectrum (Eq. 6.30)
f_tilde = L*f/U_bar;
S = (4*Kappa*L*U_bar) ./ ((2 + f_tilde.^2).^(5/6));

% Wind gust
U_gi = real( sum( sqrt(2*S*df) .* cos(2*pi*(t.'*f) + phaseAngle), 2 ) );
% Lag Simulink-kompatibel struktur (fra workspace)
wind_data.time = t';
wind_data.signals.values = U_gi;
wind_data.signals.dimensions = 1;

% Lagre i .mat for bruk i From Workspace
save('wind_data.mat', 'wind_data');

%plot(t, U_gi)

%varying wind direction
psi_wind_0 = deg2rad(180);
psi_max = psi_wind_0 + deg2rad(5);
psi_min = psi_wind_0 - deg2rad(5);
mu_w = 0.001;

% varying current 
psi_current = deg2rad(90);

% Generate vessel trajectory
[xd_setpoint, t] = setPointGen(dpMode, t_end, dt);

simin = timeseries(xd_setpoint(1:6,:),t);

% Run Simulink model and extract outputs
mdl = 'part2';
simOut = sim(mdl, 'SrcWorkspace','current');

% Extract simulation outputs
t_ship = simOut.tout;
X_ship = reshape(simOut.actual_values, 6, [])';
%F_env = simOut.env_forces;
% y_thrusters = simOut.thrust_output;
% des_thrusters = reshape(simOut.thrust_desired, 6, []);
% mag_ang_thrusters = reshape(simOut.thrust_force_angle, 10, []);


%% Plots
% Extract vessel states (positions & heading)
x_ship   = X_ship(:,1);   % surge position (x)
y_ship   = X_ship(:,2);   % sway position (y)
psi_ship = X_ship(:,3);   % heading (yaw)

% Extract vessel states (velocities)
u_ship   = X_ship(:,4);   % surge rate (u)
v_ship   = X_ship(:,5);   % sway rate (v)
r_ship   = X_ship(:,6);  % yaw rate (r)

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
figure;

% Surge (x)
subplot(3,1,1)
plot(t_ship, x_ship, 'LineWidth',1.5); hold on;
plot(t_ship, x_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('x [m]'); title('Surge (position)');
legend('Ship','Reference','Location','best');
%ylim(posXLim);

% Sway (y)
subplot(3,1,2)
plot(t_ship, y_ship, 'LineWidth',1.5); hold on;
plot(t_ship, y_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('y [m]'); title('Sway (position)');
legend('Ship','Reference','Location','best');
%ylim(posYLim);

% Heading (psi)
subplot(3,1,3);
plot(t_ship, psi_ship_deg, 'LineWidth',1.5); hold on;
plot(t_ship, psi_ref_deg,  '--', 'LineWidth',1.5);
grid on; ylabel('$\psi$ [deg]'); xlabel('Time [s]'); title('Heading');
legend('Ship','Reference','Location','best');
%ylim(headingLim);

% Rates: surge (u), sway (v), yaw rate (r)
figure; 

% Surge rate (u)
subplot(3,1,1)
plot(t_ship, u_ship, 'LineWidth',1.5); hold on;
plot(t_ship, u_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('u [m/s]'); title('Surge rate');
legend('Ship','Reference','Location','best');
%ylim([-2.1, 2.1]);

% Sway rate (v)
subplot(3,1,2)
plot(t_ship, v_ship, 'LineWidth',1.5); hold on;
plot(t_ship, v_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('v [m/s]'); title('Sway rate');
legend('Ship','Reference','Location','best');
%ylim([-2.1, 2.1]);

% Yaw rate (r)
subplot(3,1,3)
plot(t_ship, r_ship_deg, 'LineWidth',1.5); hold on;
plot(t_ship, r_ref_deg,  '--', 'LineWidth',1.5);
grid on; ylabel('r [deg/s]'); xlabel('Time [s]'); title('Yaw rate');
legend('Ship','Reference','Location','best');
%ylim([-1.5, 1.5]);

% --- XY plot (plan view): ship trajectory vs reference trajectory ---
figure('Name','XY Trajectory (Surge vs Sway)','Color','w');
plot(y_ship, x_ship, 'LineWidth',1.5); hold on;
plot(y_ref,  x_ref,  '--', 'LineWidth',1.5);
grid on; axis equal;
xlabel('y [m]'); ylabel('x [m]');
title('Plan View: x-y Trajectory');
legend('Ship','Reference','Location','best');
%xlim(posYLim); ylim(posXLim);

% Optional: start markers and heading arrows
plot(x_ship(1), y_ship(1), 'o', 'MarkerSize',6, 'HandleVisibility','off');
plot(x_ref(1),  y_ref(1),  'o', 'MarkerSize',6, 'HandleVisibility','off');

% %% === Thruster desired vs output (per thruster, same axes) ===
% % y_thrusters: output after dynamics
% % des_thrusters: desired thruster forces (set-points)
% % Converts to [Nt x nT], matches column counts, plots both traces.
% 
% % --- helper to coerce array to [Nt x N] given time vector t_ship
% coerceNtByN = @(M,tv) ...
%     ( size(M,1)==numel(tv) ) .* M + ...
%     ( size(M,2)==numel(tv) ) .* M.' + ...
%     ( (size(M,1)~=numel(tv)) && (size(M,2)~=numel(tv)) ) .* reshape(M, [], numel(tv)).';
% 
% Yt = coerceNtByN(y_thrusters, t_ship);      % output
% Td = coerceNtByN(des_thrusters, t_ship);    % desired
% 
% % If counts differ, plot the common subset
% nT = min(size(Yt,2), size(Td,2));
% Yt = Yt(:,1:nT);
% Td = Td(:,1:nT);
% 
% % Unit scaling (use kN for readability)
% scale   = 1e-3;     % set to 1 to keep N
% unitLbl = 'kN';
% 
% figure('Name','Thruster Desired vs Output','Color','w');
% for i = 1:nT
%     subplot(nT,1,i)
%     plot(t_ship, Td(:,i)*scale, '--', 'LineWidth', 1.4); hold on
%     plot(t_ship, Yt(:,i)*scale, '-',  'LineWidth', 1.6);
%     grid on
%     ylabel(sprintf('T_%d [%s]', i, unitLbl));
%     if i==1, title('Desired (set-point) vs Output (after dynamics)'); end
%     if i==nT, xlabel('Time [s]'); end
%     legend('Desired','Output','Location','best');
% end
% 
% 
% %% === Thrust allocation commands: magnitude & angle (mag_ang_thrusters) ===
% % mag_ang_thrusters is expected to be:
% %   [ T1; T2; ...; TnT;  alpha1; alpha2; ...; alphanT ]  (angles in rad)
% MA = mag_ang_thrusters;
% % Coerce to [nRows x Nt] with rows stacked by signal
% if size(MA,2) ~= numel(t_ship)
%     MA = reshape(MA, size(MA,1), []);  % ensure 2D
% end
% nRows = size(MA,1);
% nT2   = nRows/2;                        % magnitudes + angles
% if abs(nT2 - round(nT2)) > eps
%     error('mag_ang_thrusters must have 2*m rows (magnitudes, then angles).');
% end
% 
% mag_cmd = MA(1:nT2, :);                 % [nT x Nt]
% ang_cmd = MA(nT2+1:end, :);             % [nT x Nt]
% % Transpose to [Nt x nT] for plotting
% mag_cmd = mag_cmd.';                    % [Nt x nT]
% ang_cmd = ang_cmd.';                    % [Nt x nT]
% ang_deg = rad2deg(unwrap(ang_cmd));     % deg, unwrap for readability
% 
% % Magnitudes
% figure('Name','Thrust allocation: commanded magnitudes','Color','w');
% for i = 1:nT2
%     subplot(nT2,1,i)
%     plot(t_ship, mag_cmd(:,i)*scale, 'LineWidth',1.5); grid on
%     ylabel(sprintf('|T_%d| [%s]', i, unitLbl))
%     if i==1, title('Commanded thrust magnitudes'); end
% end
% xlabel('Time [s]');
% 
% % Angles
% figure('Name','Thrust allocation: commanded angles','Color','w');
% for i = 1:nT2
%     subplot(nT2,1,i)
%     plot(t_ship, ang_deg(:,i), 'LineWidth',1.5); grid on
%     ylabel(sprintf('\\alpha_%d [deg]', i))
%     if i==1, title('Commanded thrust azimuth angles'); end
% end
% xlabel('Time [s]');
