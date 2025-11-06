%% Symbolic Jacobian Calculation for EKF in DP System
% This script calculates the Jacobian matrix for the Extended Kalman Filter
% used in Dynamic Positioning systems as described in the report
% State vector: x = [xi; eta; b; nu] (15 states total)
% xi: wave-frequency states (6 states)
% eta: position and heading (3 states: x, y, psi)
% b: bias (3 states)
% nu: velocity (3 states: u, v, r)

clear; clc;

%% Define Symbolic Variables

% State vector components (15 states)
syms xi1 xi2 xi3 xi4 xi5 xi6 real  % Wave-frequency states
syms x y psi real                   % Position and heading (eta)
syms b1 b2 b3 real                  % Bias states
syms u v r real                     % Velocities (nu)

% Control inputs
syms tau_x tau_y tau_psi real       % Control forces/moments

% Parameters
syms m11 m22 m33 real               % Mass matrix diagonal elements
syms d11 d22 d33 real               % Damping matrix diagonal elements
syms Tb1 Tb2 Tb3 real               % Bias time constants

% Wave-frequency model parameters
syms omega1 omega2 omega3 real      % Natural frequencies
syms zeta1 zeta2 zeta3 real         % Damping ratios
syms Kw1 Kw2 Kw3 real               % Wave disturbance gains

% Restoring force parameters (for station keeping, typically zero)
syms g11 g22 g33 real

%% Construct State Vector
x_state = [xi1; xi2; xi3; xi4; xi5; xi6; x; y; psi; b1; b2; b3; u; v; r];

%% Define System Matrices

% Wave-frequency system matrix Aw (6x6)
Omega = diag([omega1, omega2, omega3]);
Lambda = diag([zeta1, zeta2, zeta3]);

Aw = [zeros(3,3), eye(3);
      -Omega^2, -2*Lambda*Omega];

% Rotation matrix R(psi)
R_psi = [cos(psi), -sin(psi), 0;
         sin(psi),  cos(psi), 0;
         0,         0,        1];

% Mass matrix M (3x3, diagonal)
M = diag([m11, m22, m33]);
M_inv = inv(M);

% Damping matrix D (3x3, diagonal)
D = diag([d11, d22, d33]);

% Bias time constant matrix
Tb = diag([Tb1, Tb2, Tb3]);
Tb_inv = inv(Tb);

% Restoring matrix G (3x3, typically zero for DP)
G_eta = diag([g11, g22, g33]);

%% Define State Equations f(x)

% Wave-frequency dynamics (xi_dot)
xi_vec = [xi1; xi2; xi3];
xi_dot_vec = [xi4; xi5; xi6];
f_xi = Aw * [xi_vec; xi_dot_vec];  % 6 equations

% Kinematics (eta_dot)
nu_vec = [u; v; r];
f_eta = R_psi * nu_vec;  % 3 equations

% Bias dynamics (b_dot)
b_vec = [b1; b2; b3];
f_b = -Tb_inv * b_vec;  % 3 equations (no noise term in deterministic part)

% Velocity dynamics (nu_dot)
eta_vec = [x; y; psi];
tau_vec = [tau_x; tau_y; tau_psi];
f_nu = -M_inv * D * nu_vec - M_inv * R_psi' * G_eta * eta_vec + ...
       M_inv * R_psi' * b_vec + M_inv * tau_vec;  % 3 equations

% Complete state derivative vector f(x, u)
f_x = [f_xi; f_eta; f_b; f_nu];

%% Calculate Jacobian Matrix (Phi_k = df/dx)

disp('Calculating Jacobian matrix symbolically...')
F_jacobian = jacobian(f_x, x_state);

disp('Jacobian matrix calculated successfully!')
disp(' ')

%% Display Jacobian Structure
disp('=== JACOBIAN MATRIX STRUCTURE ===')
disp('Size: 15x15')
disp(' ')
disp('Block structure:')
disp('F = [dxi_dot/dxi     |  0              |  0       |  0        ]')
disp('    [0              |  deta_dot/deta  |  0       |  deta_dot/dnu]')
disp('    [0              |  0              |  db_dot/db|  0        ]')
disp('    [dnu_dot/dxi    |  dnu_dot/deta   |  dnu_dot/db| dnu_dot/dnu]')
disp(' ')

%% Extract Key Jacobian Blocks

% Block 1: dxi_dot/dxi (6x6) - Wave-frequency dynamics
J_xi_xi = F_jacobian(1:6, 1:6);

% Block 2: deta_dot/deta (3x3) - Position kinematics w.r.t. eta
J_eta_eta = F_jacobian(7:9, 7:9);

% Block 3: deta_dot/dnu (3x3) - Position kinematics w.r.t. velocity
J_eta_nu = F_jacobian(7:9, 13:15);

% Block 4: db_dot/db (3x3) - Bias dynamics
J_b_b = F_jacobian(10:12, 10:12);

% Block 5: dnu_dot/deta (3x3) - Velocity dynamics w.r.t. position
J_nu_eta = F_jacobian(13:15, 7:9);

% Block 6: dnu_dot/db (3x3) - Velocity dynamics w.r.t. bias
J_nu_b = F_jacobian(13:15, 10:12);

% Block 7: dnu_dot/dnu (3x3) - Velocity dynamics w.r.t. velocity
J_nu_nu = F_jacobian(13:15, 13:15);

%% Display Individual Blocks
disp('=== KEY JACOBIAN BLOCKS ===')
disp(' ')

disp('1. Wave-frequency block (dxi_dot/dxi):')
disp('This is simply Aw matrix:')
pretty(J_xi_xi)
disp(' ')

disp('2. Position kinematics w.r.t. heading (deta_dot/deta):')
disp('This contains dR(psi)/dpsi * nu:')
pretty(J_eta_eta)
disp(' ')

disp('3. Position kinematics w.r.t. velocity (deta_dot/dnu):')
disp('This is R(psi):')
pretty(J_eta_nu)
disp(' ')

disp('4. Bias dynamics (db_dot/db):')
disp('This is -inv(Tb):')
pretty(J_b_b)
disp(' ')

disp('5. Velocity dynamics w.r.t. position (dnu_dot/deta):')
disp('This contains gravity/restoring terms:')
pretty(J_nu_eta)
disp(' ')

disp('6. Velocity dynamics w.r.t. bias (dnu_dot/db):')
disp('This is inv(M) * R^T(psi):')
pretty(J_nu_b)
disp(' ')

disp('7. Velocity dynamics w.r.t. velocity (dnu_dot/dnu):')
disp('This is -inv(M) * D:')
pretty(J_nu_nu)
disp(' ')
