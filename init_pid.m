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

%% Nonlinear PID Pole -placement 
omega_b1 = 1; omega_b2 = 1; omega_b6 = 1;
Omega_b = diag([omega_b1, omega_b2, omega_b6]);

zeta_1 = 1; zeta_2 = 1; zeta_6 = 1;
Zeta = diag([zeta_1, zeta_2, zeta_6]);

omega_n = zeros(1,3);
for i = length(Omega_b)
    omega_n(i) = Omega_b(i) / (sqrt(1 - 2*Zeta(i)^2 + sqrt(4*Zeta(i)^4 - 4*zeta(i)^2 + 2)));
end

Omega_n = diag(omega_n);

Kp = M*Omega_n^2;

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