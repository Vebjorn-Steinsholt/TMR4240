%% Plots
% Extract simulation outputs
t_ship = simOut.tout;
X_ship = reshape(simOut.actual_values, 6, [])';
X_obs = reshape(simOut.estimated_values, 6, [])';
% Extract vessel states (positions & heading)
x_ship   = X_ship(:,1);   % surge position (x)
y_ship   = X_ship(:,2);   % sway position (y)
psi_ship = X_ship(:,3);   % heading (yaw)

% Extract vessel states (velocities)
u_ship   = X_ship(:,4);   % surge rate (u)
v_ship   = X_ship(:,5);   % sway rate (v)
r_ship   = X_ship(:,6);  % yaw rate (r)

%For estimated position
% Extract observer (estimated) states (positions & heading)
x_obs   = X_obs(:,1);   % estimated surge position (x)
y_obs   = X_obs(:,2);   % estimated sway position (y)
psi_obs = X_obs(:,3);   % estimated heading (yaw)

% Extract observer (estimated) velocities
u_obs   = X_obs(:,4);   % estimated surge rate (u)
v_obs   = X_obs(:,5);   % estimated sway rate (v)
r_obs   = X_obs(:,6);   % estimated yaw rate (r)

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
psi_obs_u   = unwrap(psi_obs);
psi_obs_deg = rad2deg(psi_obs_u);   % convert to degrees for plotting

% Convert yaw rates to deg/s for readability (matches your psi in deg)
r_ship_deg = rad2deg(r_ship);
r_ref_deg  = rad2deg(r_ref);
r_obs_deg = rad2deg(r_obs);

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
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Sim1
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


%%%%%%%%%%%%%%%Sim4
figure;
% Surge (x)
subplot(3,1,1)
plot(t_ship, x_ship, 'LineWidth',1.5); hold on;
plot(t_ship, x_obs, "-", 'LineWidth',1.5); hold on;
plot(t_ship, x_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('x [m]'); title('Surge (position)');
legend('Ship',"Observer",'Reference','Location','best');
%ylim(posXLim);

% Sway (y)
subplot(3,1,2)
plot(t_ship, y_ship, 'LineWidth',1.5); hold on;
plot(t_ship, y_obs, 'LineWidth',1.5); hold on;
plot(t_ship, y_ref,  '--', 'LineWidth',1.5);
grid on; ylabel('y [m]'); title('Sway (position)');
legend('Ship',"Observer",'Reference','Location','best');
%ylim(posYLim);

% Heading (psi)
subplot(3,1,3);
plot(t_ship, psi_ship_deg, 'LineWidth',1.5); hold on;
plot(t_ship, psi_obs_deg, 'LineWidth',1.5); hold on;
plot(t_ship, psi_ref_deg,  '--', 'LineWidth',1.5);
grid on; ylabel('$\psi$ [deg]'); xlabel('Time [s]'); title('Heading');
legend('Ship',"Observer",'Reference','Location','best');
%ylim(headingLim);

%% FIGURE: Velocity Comparison (u, v, r)
figure;

% Surge rate (u)
subplot(3,1,1)
plot(t_ship, u_ship, 'LineWidth',1.5); hold on;
plot(t_ship, u_obs, "-", 'LineWidth',1.5); hold on;
plot(t_ship, u_ref,  '--', 'LineWidth',1.5);
grid on;
ylabel('u [m/s]');
title('Surge Velocity');
legend('Ship', 'Observer', 'Reference', 'Location', 'best');

% Sway rate (v)
subplot(3,1,2)
plot(t_ship, v_ship, 'LineWidth',1.5); hold on;
plot(t_ship, v_obs, "-", 'LineWidth',1.5); hold on;
plot(t_ship, v_ref,  '--', 'LineWidth',1.5);
grid on;
ylabel('v [m/s]');
title('Sway Velocity');
legend('Ship', 'Observer', 'Reference', 'Location', 'best');

% Yaw rate (r)
subplot(3,1,3)
plot(t_ship, r_ship_deg, 'LineWidth',1.5); hold on;
plot(t_ship, r_obs_deg, "-", 'LineWidth',1.5); hold on;
plot(t_ship, r_ref_deg,  '--', 'LineWidth',1.5);
grid on;
ylabel('r [deg/s]');
xlabel('Time [s]');
title('Yaw Rate');
legend('Ship', 'Observer', 'Reference', 'Location', 'best');

%% Plot tau_d (desired forces) vs thrustDynamic (actual thrusts)
figure('Name','Desired vs Actual Thrust Forces','Color','w');

% Extract data
t_tau_d = simOut.tau_d.Time;
data_tau_d = simOut.tau_d.Data;              % [30001 x 6]

t_thrustDynamic = simOut.thrustDynamic.Time;
data_thrustDynamic = simOut.thrustDynamic.Data;  % [30001 x 6]

% --- Combined plot: tau_d and thrustDynamic ---
for i = 1:size(data_tau_d,2)
    subplot(size(data_tau_d,2),1,i)
    plot(t_tau_d, data_tau_d(:,i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t_thrustDynamic, data_thrustDynamic(:,i), 'LineWidth', 1.5);
    grid on;
    ylabel(sprintf('DOF %d [N or N·m]', i));
    if i == 1
        title('Desired Control Forces (\tau_d) vs Thrust Dynamics');
    end
    if i == size(data_tau_d,2)
        xlabel('Time [s]');
    end
    legend('\tau_d (desired)','ThrustDynamic (actual)','Location','best');
end

%% Plot thrust allocation separately
figure('Name','Thrust Allocation','Color','w');

t_thrustAlloc = simOut.thrustAlloc.Time;
data_thrustAlloc = squeeze(simOut.thrustAlloc.Data);   % [5 x 1 x 30001] → [5 x 30001]
data_thrustAlloc = data_thrustAlloc.';                 % transpose to [30001 x 5]

plot(t_thrustAlloc, data_thrustAlloc, 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Force [N]');
title('Thrust Allocation (per thruster)');
legend(compose('Thruster %d', 1:size(data_thrustAlloc,2)), 'Location','best');
