%% LQR Controller Design for Surface Ocean Vehicle (SOV)
% Based on Fossen's Simplified Maneuvering Model for DP Ships
% Degrees of Freedom: Surge (x), Sway (y), and Yaw (psi)
%
% Author: Control Systems Example
% Date: 2025

clear all; close all; clc;

%% ========================================================================
% PART 1: DEFINE VESSEL PARAMETERS (Fossen's Notation)
% ========================================================================

% Mass and inertia parameters
m = 100;        % Mass [kg]
Iz = 50;        % Moment of inertia about z-axis [kg*m^2]
xg = 0;         % Center of gravity x-coordinate [m]

% Added mass terms (due to water displaced)
Xu_dot = -10;   % Added mass in surge [kg]
Yv_dot = -20;   % Added mass in sway [kg]
Yr_dot = 0;     % Added mass coupling term [kg*m]
Nv_dot = 0;     % Added mass coupling term [kg*m]
Nr_dot = -5;    % Added mass in yaw [kg*m^2]

% Linear damping coefficients
Xu = -5;        % Linear surge damping [kg/s]
Yv = -10;       % Linear sway damping [kg/s]
Yr = 0;         % Linear sway-yaw coupling damping [kg*m/s]
Nv = 0;         % Linear yaw-sway coupling damping [kg*m/s]
Nr = -2;        % Linear yaw damping [kg*m^2/s]

% System inertia matrix M (including added mass)
M11 = m - Xu_dot;
M22 = m - Yv_dot;
M23 = m*xg - Yr_dot;
M32 = m*xg - Nv_dot;
M33 = Iz - Nr_dot;

M = [M11  0    0;
     0    M22  M23;
     0    M32  M33];

% Linear damping matrix D
D = -[Xu  0   0;
      0   Yv  Yr;
      0   Nv  Nr];

% Display system matrices
fprintf('=== Vessel Parameters ===\n');
fprintf('Mass matrix M:\n'); disp(M);
fprintf('Damping matrix D:\n'); disp(D);

%% ========================================================================
% PART 2: NONLINEAR DYNAMICS (Fossen's 3-DOF Model)
% ========================================================================

% State vector: x = [u, v, r, x, y, psi]'
% where:
%   u   = surge velocity (body frame) [m/s]
%   v   = sway velocity (body frame) [m/s]
%   r   = yaw rate (body frame) [rad/s]
%   x   = north position (earth frame) [m]
%   y   = east position (earth frame) [m]
%   psi = heading angle (earth frame) [rad]
%
% Control input: u_c = [tau_u, tau_v, tau_r]'
%   tau_u = surge force [N]
%   tau_v = sway force [N]
%   tau_r = yaw moment [N*m]

% Nonlinear state-space function
% dx/dt = f(x, u_c)
nonlinear_dynamics = @(x, u_c) sov_dynamics(x, u_c, M, D);

%% ========================================================================
% PART 3: LINEARIZATION AROUND OPERATING POINT
% ========================================================================

% Define equilibrium/operating point
% For DP: stationary vessel at origin with zero velocities
x_eq = [0; 0; 0; 0; 0; 0];  % [u, v, r, x, y, psi]
u_eq = [0; 0; 0];             % [tau_u, tau_v, tau_r]

fprintf('\n=== Linearization ===\n');
fprintf('Operating point (equilibrium):\n');
fprintf('  State x_eq = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]''\n', x_eq);
fprintf('  Input u_eq = [%.2f, %.2f, %.2f]''\n', u_eq);

% Compute Jacobian matrices A and B using symbolic differentiation
syms u v r x y psi tau_u tau_v tau_r real
x_sym = [u; v; r; x; y; psi];
u_sym = [tau_u; tau_v; tau_r];

% Symbolic dynamics
f_sym = sov_dynamics_symbolic(x_sym, u_sym, M, D);

% Compute Jacobians
A_sym = jacobian(f_sym, x_sym);
B_sym = jacobian(f_sym, u_sym);

% Evaluate at equilibrium point
A = double(subs(A_sym, [x_sym; u_sym], [x_eq; u_eq]));
B = double(subs(B_sym, [x_sym; u_sym], [x_eq; u_eq]));

fprintf('\nLinearized A matrix (6x6):\n'); disp(A);
fprintf('Linearized B matrix (6x3):\n'); disp(B);

% Check controllability
Co = ctrb(A, B);
rank_Co = rank(Co);
fprintf('\nControllability check:\n');
fprintf('  Rank of controllability matrix: %d (should be %d)\n', rank_Co, size(A,1));
if rank_Co == size(A,1)
    fprintf('  System is CONTROLLABLE ✓\n');
else
    fprintf('  System is NOT fully controllable ✗\n');
end

%% ========================================================================
% PART 4: LQR CONTROLLER DESIGN
% ========================================================================

fprintf('\n=== LQR Controller Design ===\n');

% Define weighting matrices Q and R
% Q: State cost matrix (penalizes deviation from equilibrium)
% R: Control effort cost matrix (penalizes control input magnitude)

% Weighting strategy:
% - Heavily penalize position errors (x, y, psi)
% - Moderately penalize velocities (u, v, r)
% - Moderate control effort penalty

Q = diag([1, 1, 1, 100, 100, 50]);  % Weight on [u, v, r, x, y, psi]
R = diag([1, 1, 1]);                 % Weight on [tau_u, tau_v, tau_r]

fprintf('State weighting matrix Q:\n'); disp(Q);
fprintf('Control weighting matrix R:\n'); disp(R);

% Compute LQR gain matrix K
% Control law: u = -K*x (for regulation to origin)
[K, S, e] = lqr(A, B, Q, R);

fprintf('LQR Gain matrix K (3x6):\n'); disp(K);
fprintf('Closed-loop eigenvalues:\n'); disp(e);

% Check stability
if all(real(e) < 0)
    fprintf('Closed-loop system is STABLE ✓\n');
else
    fprintf('Closed-loop system is UNSTABLE ✗\n');
end

%% ========================================================================
% PART 5: SIMULATION OF CONTROLLED SYSTEM
% ========================================================================

fprintf('\n=== Simulation ===\n');

% Initial condition (displaced from equilibrium)
x0 = [0.5; 0.3; 0.1; 10; 5; 0.2];  % Initial state

% Simulation time
tspan = [0 50];  % 50 seconds

% Closed-loop dynamics with LQR control
% dx/dt = f(x, u) where u = -K*x
closed_loop_dynamics = @(t, x) nonlinear_dynamics(x, -K*x);

% Solve ODE
fprintf('Running simulation from t=%.1f to t=%.1f seconds...\n', tspan(1), tspan(2));
[t, x_sim] = ode45(closed_loop_dynamics, tspan, x0);

% Compute control inputs
u_sim = -(K * x_sim')';

%% ========================================================================
% PART 6: VISUALIZATION
% ========================================================================

fprintf('\n=== Plotting Results ===\n');

% Plot 1: States vs Time
figure('Name', 'LQR Control - State Response', 'Position', [100 100 1200 800]);

subplot(3,2,1);
plot(t, x_sim(:,1), 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('u [m/s]');
title('Surge Velocity');

subplot(3,2,2);
plot(t, x_sim(:,2), 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('v [m/s]');
title('Sway Velocity');

subplot(3,2,3);
plot(t, x_sim(:,3), 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('r [rad/s]');
title('Yaw Rate');

subplot(3,2,4);
plot(t, x_sim(:,4), 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('x [m]');
title('North Position');

subplot(3,2,5);
plot(t, x_sim(:,5), 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('y [m]');
title('East Position');

subplot(3,2,6);
plot(t, x_sim(:,6)*180/pi, 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('\psi [deg]');
title('Heading Angle');

% Plot 2: Control Inputs vs Time
figure('Name', 'LQR Control - Control Inputs', 'Position', [150 150 1000 400]);

subplot(1,3,1);
plot(t, u_sim(:,1), 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('\tau_u [N]');
title('Surge Force');

subplot(1,3,2);
plot(t, u_sim(:,2), 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('\tau_v [N]');
title('Sway Force');

subplot(1,3,3);
plot(t, u_sim(:,3), 'LineWidth', 2);
grid on; xlabel('Time [s]'); ylabel('\tau_r [N·m]');
title('Yaw Moment');

% Plot 3: Trajectory (Top View)
figure('Name', 'LQR Control - Vessel Trajectory', 'Position', [200 200 600 600]);
plot(x_sim(:,5), x_sim(:,4), 'b-', 'LineWidth', 2); hold on;
plot(x_sim(1,5), x_sim(1,4), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(0, 0, 'rx', 'MarkerSize', 15, 'LineWidth', 3);
grid on; axis equal;
xlabel('East (y) [m]'); ylabel('North (x) [m]');
title('Vessel Trajectory (Top View)');
legend('Trajectory', 'Start', 'Target', 'Location', 'best');

fprintf('Simulation complete!\n');

%% ========================================================================
% FUNCTION DEFINITIONS
% ========================================================================

function dx = sov_dynamics(x, u_c, M, D)
    % Nonlinear dynamics for 3-DOF SOV (numerical version)
    % x = [u, v, r, x, y, psi]'
    % u_c = [tau_u, tau_v, tau_r]'
    
    % Extract states
    u = x(1);
    v = x(2);
    r = x(3);
    psi = x(6);
    
    % Coriolis-centripetal matrix (velocity-dependent)
    % C(nu) represents the coupling between velocities
    m = M(1,1);
    m_v = M(2,2);
    m_r = M(2,3);
    
    C = [0,  0,  -m_v*v - m_r*r;
         0,  0,   m*u;
         0, -m*u,  0];
    
    % Rotation matrix (body to earth frame)
    J = [cos(psi), -sin(psi), 0;
         sin(psi),  cos(psi), 0;
         0,         0,        1];
    
    % Body-frame velocities
    nu = [u; v; r];
    
    % Dynamics in body frame: M*nu_dot + C*nu + D*nu = tau
    % nu_dot = M^(-1) * (tau - C*nu - D*nu)
    nu_dot = M \ (u_c - C*nu - D*nu);
    
    % Kinematics: eta_dot = J*nu
    eta_dot = J * nu;
    
    % Complete state derivative
    dx = [nu_dot; eta_dot];
end

function f = sov_dynamics_symbolic(x, u_c, M, D)
    % Symbolic version for Jacobian computation
    % x = [u, v, r, x, y, psi]'
    
    u = x(1);
    v = x(2);
    r = x(3);
    psi = x(6);
    
    % Coriolis matrix (symbolic)
    m = M(1,1);
    m_v = M(2,2);
    m_r = M(2,3);
    
    C = [0,  0,  -m_v*v - m_r*r;
         0,  0,   m*u;
         0, -m*u,  0];
    
    % Rotation matrix
    J = [cos(psi), -sin(psi), 0;
         sin(psi),  cos(psi), 0;
         0,         0,        1];
    
    nu = [u; v; r];
    
    % Dynamics
    nu_dot = M \ (u_c - C*nu - D*nu);
    eta_dot = J * nu;
    
    f = [nu_dot; eta_dot];
end