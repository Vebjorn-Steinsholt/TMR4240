%% Generate set-points for simulation tests
spMode = 3; % 1: [0 0 0], 2: single waypoint, 3: multi waypoint

% Position and attitude ref model, based on Fossen, 2021
omega11 = 0.05; omega22 = 0.035; omega33 = 0.04;        % natural freq.
zeta11 = 1; zeta22 = 1; zeta33 = 1;                 % relative damping coeff.

Omega = diag([omega11 omega22 omega33]);
Delta = diag([zeta11 zeta22 zeta33]);

% Third order state-space representation
Ad = [zeros(3)                       eye(3)                   zeros(3);
      zeros(3)                     zeros(3)                   eye(3);
      -Omega^3 -(2.*Delta + eye(3))*Omega^2 -(2.*Delta + eye(3))*Omega];

Bd = [zeros(3) zeros(3) Omega^3]';

f_pa = @(xd, r) Ad*xd + Bd*r;

%% Simulation
% Simulation parameters
dt = 0.01;          % Time step
t_final = 3000;      % Final time
t = 0:dt:t_final;   % Time vector
N = length(t);


% Set points reference model
switch spMode
    case 1
        waypoints = [0 0 0]';
    case 2
        waypoints = [10 10 ssa((3*pi)/2)]';       % Constant reference position and heading
    case 3
        eta_sp0 = [0 0 0]';
        eta_sp1 = [50 0 0]';
        eta_sp2 = [50 -50 0]'; 
        eta_sp3 = [50 -50 ssa(-pi/4)]';
        eta_sp4 = [0 -50 ssa(-pi/4)]';
        eta_sp5 = [0 0 0]';
        waypoints = [eta_sp0 eta_sp1 eta_sp2 eta_sp3 eta_sp4 eta_sp5];
    otherwise
        disp('Choose a valid DP mode');
end

N_wp = size(waypoints, 2);
wp_interval = floor(N/N_wp-1);

xd0 = [0 0 0 0 0 0 0 0 0]';   % Initial states
% xd = [eta_d eta_d_dot eta_d_ddot]';
% eta_d = [x y psi]';
% eta_d_dot = [u v r]';
% eta_d_ddot = [u_dot v_dot r_dot];

xd = zeros(9, N);   % State trajectory
xd(:,1) = xd0;      % Initial condition

% Main simulation loop
for k = 1:N-1
    wp_idx = min(floor((k-1)/wp_interval)+1, N_wp);
    r = waypoints(:,wp_idx);

    xd(:,k+1) = rk4(@(x) f_pa(x, r), dt, xd(:,k));
end

%% Plot results

% x and y over time
figure;
plot(t, xd(1,:), 'r', 'LineWidth', 1.5); hold on;
plot(t, xd(2,:), 'g', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Position [m]');
legend('$x$','$y$','Interpreter','latex');
title('Position vs. Time');

%% psi over time (in radians)
psi = wrapToPi(xd(3,:));   % keep angle between -pi and pi

figure;
plot(t, psi, 'b', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]');
ylabel('Heading $\psi$ [rad]','Interpreter','latex');
legend('$\psi$','Interpreter','latex');
title('Heading vs. Time');


% XY trajectory
figure;
plot(xd(2,:), xd(1,:), 'k', 'LineWidth', 1.5);
grid on; axis equal;
xlabel('y [m]');
ylabel('x [m]');
title('XY Trajectory');