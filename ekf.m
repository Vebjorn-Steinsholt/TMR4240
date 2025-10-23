%% EKF

%% states init
x0 = zeros(15, 1); % 15 states
P0 = eye(15);


%% predict


%% update


%% Matrices

% omega_0 = pi/10 -> from the wave freq model
omega_0 = pi/10;
omega = diag([omega_0, omega_0, omega_0]);

zeta = 0.1;
gam = diag([zeta, zeta, zeta]);

K_w = diag([1, 1, 1]); % change this

% A_w = [0, I;
%   -omg^2, -2gamma omg]

I = eye(3); % Identity matrix for the state dimension
A_w = [zeros(3), I; -omega^2, -2 * gam * omega];

% E_w = [0, K_w]
E_w = [zeros(3); K_w];

% C_w = [0, I]
C_w = [zeros(3), I];