%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% init()                                                                  %
%                                                                         %              
% Set initial parameters for part1.slx and part2.slx                      %
%                                                                         %
% Created:      2018.07.12	Jon Bjørnø                                    %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;

load('thrusters_sup.mat')
load('supply.mat');
load('supplyABC.mat');

% Initial position x, y, z, phi, theta, psi
eta0 = [0,0,0,0,0,0]';
% Initial velocity u, v, w, p, q, r
nu0 = [0,0,0,0,0,0]';

windAngle = 0;

% PID parameters
Kp = diag([7.8e5   9.8e5   1.5e8]);
Ki = diag([8.5e3 7.5e3 3.5e4]);
Kd = diag([9.3e5 7e5 4e6]);
Kp = diag([7.8e5   5e5   1.5e8]);
Ki = diag([8.5e3 7.5e3 3.5e4]);
Kd = diag([9.3e5 7e5 4e6]);

t_end = 200;
dt = 0.01;

% Generate vessel trajectory
%[xd_setpoint, t] = setPointGen(dpMode, t_end, dt);

%simin = timeseries(xd_setpoint(1:3,:),t);

% Run Simulink model and extract outputs
mdl = 'part1';
%simOut = sim(mdl, 'SrcWorkspace','current');

% Extract simulation outputs
%t_ship = simOut.tout;
%X_ship = reshape(simOut.ship_states, 12, [])';
%F_env = simOut.env_forces;

%Non-linear Observer
% Mass matrix (M = MRB + MA)
M = [7.0184e6      0              0;
     0        8.5464e6   -4.4678e7;
     0       -4.5028e5    4.0504e9];

% Damping matrix (D)
D = [2.6486e5     0           0;
     0        8.8164e5       0;
     0             0     3.3774e8];
M_inv = M\eye(3);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
currentMode = 5;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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
zeta_ni = 1;
zeta_i = 0.1;
w_i = pi/10;
w_ci = 0.13*pi;
%diagonla values
k11 = -2*(zeta_ni-zeta_i)*w_ci/w_i;
k12 = 2*w_i*(zeta_ni-zeta_i);
k2 = w_ci;
%k3 = 7e5;
%k4 = k3*10;
%Observer Gains matrixes
K1 = [k11 0 0;
      0 k11 0;
      0 0 k11;
      k12 0 0;
      0 k12 0;
      0 0 k12;];
K2 = [k2 0 0;
      0 k2*4 0;
      0 0 k2];
K4 = [7.0184e6*100      0              0;
     0        10.5464e6*10    0;
     0       0   4.0504e9];
K3 = [7.0184e6*0.1      0              0;
     0        10.5464e6*0.1   0;
     0       0   4.0504e9*0.1];

%Harris wind spectrum
% parameters
dt = 0.001;
t = 0:dt:500;

Kappa = 0.0026; 
L = 1800; 
U_bar = 10;

f = linspace(0.001, 1, 200);
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

