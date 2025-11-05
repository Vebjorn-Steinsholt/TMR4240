function [Kp, Ki, Kd] = init_pid(vessel)
%INIT_PID Summary of this function goes here
%   Detailed explanation goes here

%% Vessel
U = 0; % Assuming forward speed U = 0
psi = 0; % Heading angle
beta = deg2rad(135); % Wave direction

plotFlag = 1;

omega_p = 0.8; % Peak frequencies [rad/s]

% Compute added mass and 
g = 9.81; 
omega_p = omega_p - (omega_p^2 / g) * U * cos(beta);
vessel = computeManeuveringModel(vessel, omega_p, plotFlag);

% Mass-inertia matrix
idx = [1 2 6];
MRB = vessel.MRB(idx, idx);
MA = vessel.A_eq(idx, idx, 1 , 1);
M = MRB + MA;

D = vessel.B_eq(idx, idx, 1 , 1);

% M = [7.0184e6      0              0;
%      0        8.5464e6   -4.4678e7;
%      0       -4.5028e5    4.0504e9];
% 
% % Damping matrix (D)
% D = [2.6486e5     0           0;
%      0        8.8164e5       0;
%      0             0     3.3774e8];
% M_inv = M\eye(3);

%% Nonlinear PID Pole-placement 
% Computing PID-gains using Algorithem 15.2 from (Fossen, 2021)
omega_b1 = 0.12; omega_b2 = 0.12; omega_b6 = 0.08;
omega_b = [omega_b1, omega_b2, omega_b6];
Omega_b = diag(omega_b);

zeta_pid1 = 0.80; zeta_pid2 = 0.80; zeta_pid6 = 1;
zeta_pid = [zeta_pid1, zeta_pid2, zeta_pid6];
Zeta_pid = diag(zeta_pid);

omega_n = zeros(1,3);
for i = 1:length(omega_n)
    omega_n(i) = omega_b(i) / ( sqrt(1 - 2*zeta_pid(i)^2 + sqrt(4*zeta_pid(i)^4 - 4*zeta_pid(i)^2 + 2) ) );
end
Omega_n = diag(omega_n);

% Assuming roll, pitch and yaw is small => J_Theta(eta) = I
Kp = M*Omega_n^2;

Kd = 2.*M*Zeta_pid*Omega_n + D; 

Ki = 0.10*Kp*Omega_n;

% %% Tuneable gains
% % Proportional gains
% Kp = diag([400e3, 400e3, 800e6]);
% 
% % Derivative gains
% Kd = diag([800e3, 800e3, 1600e6]);
% 
% % Integral gains
% Ki = diag([20e3, 20e3, 40e6]);




end